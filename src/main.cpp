// Minimal USB Mass Storage proof-of-concept for ESP32-S2/S3 boards.
//
// Mounts the SD card and exposes it to a USB host as a removable SCSI disk,
// using the exact device descriptor and MSC callbacks from Launcher
// (src/massStorage.cpp: PR #297 bDeviceClass fix + issue #424 partial/non-zero
// offset READ10/WRITE10 fix). Everything else in Launcher (display, menus,
// keyboard, WiFi, OTA, WebUI) is gone, so a host-side mount failure can be
// attributed to the USB MSC path itself instead of to the surrounding app.
//
// The only intentional difference from Launcher is the iSerialNumber string
// ("MSC-POC"), so host logs can tell the two firmwares apart.
//
// Firmware does not need the display to be initialized. It logs to
// /msc-poc.log on the SD card root; see README.md for what is recorded.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <cstdarg>
#include <cstring>

#if !(CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3)
#error "This PoC only supports ESP32-S2/S3 (single full-speed USB-OTG controller)."
#endif

#include <esp_private/usb_phy.h>
#include <hal/clk_gate_ll.h>
#include <soc/periph_defs.h>
#include <soc/rtc_cntl_reg.h>
#include <tusb.h>

#include "msc_transfer.h"

#ifndef SDCARD_CS
#error "Board env must define SDCARD_CS/SCK/MISO/MOSI."
#endif

// Set by CI to the release tag (e.g. "v0.1.0"), so a flashed image can be
// traced back to a release from its log. "dev" for local builds.
#ifndef POC_VERSION
#define POC_VERSION ""
#endif
// POC_VERSION is empty when no version was passed in.
constexpr const char *kVersion = sizeof(POC_VERSION) > 1 ? POC_VERSION : "dev";

// ---------------------------------------------------------------------------
// Logging
//
// MSC callbacks run inside tud_task() (the usb_msc task), and so does the log
// flush. Nothing here touches the SD card from another task, so log writes can
// never interleave with raw sector reads/writes of the exported volume.
//
// The log is also never written while the host has the volume mounted: the
// host caches FAT/directory sectors, and writing the file behind its back could
// corrupt or confuse the very filesystem we are testing. Events are buffered in
// RAM and flushed when the host is not using the card (i.e. always, in the
// failure cases we care about) or when the host asks for an eject.
//
// READ10/WRITE10 are aggregated (a host FAT scan can issue thousands); other
// SCSI opcodes go into a 256-bucket histogram; lifecycle events and errors are
// kept verbatim in a bounded text buffer.
// ---------------------------------------------------------------------------

namespace {

constexpr const char *kLogPath = "/msc-poc.log";
constexpr uint32_t kIdleFlushMs = 400;
constexpr uint32_t kMaxFlushIntervalMs = 2000;
constexpr size_t kEventBufSize = 8192;

char s_eventBuf[kEventBufSize];
size_t s_eventLen = 0;
bool s_eventOverflow = false;

uint32_t s_opCount[256] = {};

uint32_t s_readCount = 0, s_readBytes = 0, s_readFirstLba = 0, s_readLastLba = 0;
uint32_t s_writeCount = 0, s_writeBytes = 0, s_writeFirstLba = 0, s_writeLastLba = 0;

uint32_t s_errorCount = 0;
uint32_t s_lastMscActivityMs = 0;
uint32_t s_lastFlushMs = 0;
bool s_hostMounted = false;
bool s_logReady = false; // set once the SD card is mounted

// Full-size stack buffer: SdFat raw transfers want sector-aligned 512 byte buffers.
uint8_t s_sectorBuf[512];

bool sdReadSector(void *ctx, uint32_t sector, uint8_t *dst) {
    (void)ctx;
    return SD.readRAW(dst, sector);
}

bool sdWriteSector(void *ctx, uint32_t sector, const uint8_t *src) {
    (void)ctx;
    return SD.writeRAW(const_cast<uint8_t *>(src), sector);
}

// Must outlive setup(): SdFat keeps the SPIClass reference for every later transfer.
SPIClass s_sdcardSPI;

void appendEvent(const char *fmt, ...) {
    char line[160];
    int n = snprintf(line, sizeof(line), "[%8lu] ", (unsigned long)millis());
    if (n < 0) return;
    size_t used = static_cast<size_t>(n) < sizeof(line) ? static_cast<size_t>(n) : sizeof(line) - 1;

    va_list args;
    va_start(args, fmt);
    vsnprintf(line + used, sizeof(line) - used, fmt, args);
    va_end(args);

    const size_t add = strlen(line);
    if (s_eventLen + add + 1 >= kEventBufSize) {
        s_eventOverflow = true;
        return;
    }
    memcpy(s_eventBuf + s_eventLen, line, add);
    s_eventLen += add;
    s_eventBuf[s_eventLen++] = '\n';
}

void noteActivity() { s_lastMscActivityMs = millis(); }

// Writes the accumulated counters/events and resets them. Only ever called from
// the usb_msc task. Skipped while the host has the volume mounted unless
// `force` is set (host-initiated eject).
void flushLog(const char *reason, bool force = false) {
    if (!s_logReady) return;
    if (s_hostMounted && !force) return;
    if (s_eventLen == 0 && s_errorCount == 0 && s_readCount == 0 && s_writeCount == 0) return;

    File f = SD.open(kLogPath, FILE_APPEND, true);
    if (!f) return;

    f.printf(
        "---- flush %lu (%s) ----\n"
        "counters: read=%lu (%lu B, lba %lu..%lu) write=%lu (%lu B, lba %lu..%lu) errors=%lu\n",
        (unsigned long)millis(),
        reason,
        (unsigned long)s_readCount,
        (unsigned long)s_readBytes,
        (unsigned long)s_readFirstLba,
        (unsigned long)s_readLastLba,
        (unsigned long)s_writeCount,
        (unsigned long)s_writeBytes,
        (unsigned long)s_writeFirstLba,
        (unsigned long)s_writeLastLba,
        (unsigned long)s_errorCount
    );

    for (int op = 0; op <= 0xFF; ++op) {
        if (op == SCSI_CMD_READ_10 || op == SCSI_CMD_WRITE_10) continue;
        if (s_opCount[op]) f.printf("scsi 0x%02X: %lu\n", op, (unsigned long)s_opCount[op]);
    }

    if (s_eventLen) f.write(reinterpret_cast<const uint8_t *>(s_eventBuf), s_eventLen);
    if (s_eventOverflow) f.println("[event buffer full, earlier events dropped]");
    f.println();
    f.close();

    memset(s_opCount, 0, sizeof(s_opCount));
    s_eventLen = 0;
    s_eventOverflow = false;
    s_readCount = s_readBytes = s_readFirstLba = s_readLastLba = 0;
    s_writeCount = s_writeBytes = s_writeFirstLba = s_writeLastLba = 0;
    s_errorCount = 0;
    s_lastFlushMs = millis();
}

void logHeader() {
    File f = SD.open(kLogPath, FILE_APPEND, true);
    if (!f) return;
    f.printf(
        "\n==== %s USB MSC PoC %s ====\n"
        "mcu %s, sd cs=%d sck=%d miso=%d mosi=%d\n"
        "sd: %lu sectors x %lu bytes = %lu MB\n",
        DEVICE_NAME,
        kVersion,
        ESP.getChipModel(),
        SDCARD_CS,
        SDCARD_SCK,
        SDCARD_MISO,
        SDCARD_MOSI,
        (unsigned long)SD.numSectors(),
        (unsigned long)SD.sectorSize(),
        (unsigned long)(((uint64_t)SD.numSectors() * SD.sectorSize()) >> 20)
    );
    f.close();
}

// ---------------------------------------------------------------------------
// USB MSC
// ---------------------------------------------------------------------------

usb_phy_handle_t s_usbPhy = nullptr;
TaskHandle_t s_usbTask = nullptr;
bool s_usbStarted = false;
volatile bool s_ejectRequested = false;

constexpr uint8_t kUsbItfMsc = 0;
constexpr uint8_t kUsbEpOut = 0x01;
constexpr uint8_t kUsbEpIn = 0x81;
constexpr uint8_t kUsbRhPort = 0;      // S2/S3 have a single OTG controller
constexpr uint16_t kUsbEpSize = 64;    // full speed bulk endpoints
constexpr uint16_t kUsbPowerMa = 500;

// Identical to Launcher's kDeviceDescriptor: class/subclass/protocol are 0x00
// and described at interface level. This is the PR #297 macOS fix.
constexpr tusb_desc_device_t kDeviceDescriptor = {
    sizeof(tusb_desc_device_t),
    TUSB_DESC_DEVICE,
    0x0200,
    0x00, // bDeviceClass    - defined at interface level
    0x00, // bDeviceSubClass - defined at interface level
    0x00, // bDeviceProtocol - defined at interface level
    CFG_TUD_ENDPOINT0_SIZE,
    0x303A, // Espressif VID
    0x1001,
    0x0100,
    0x01, // iManufacturer
    0x02, // iProduct
    0x03, // iSerialNumber
    0x01, // bNumConfigurations
};

constexpr uint8_t kConfigDescriptor[] = {
    TUD_CONFIG_DESCRIPTOR(
        1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, kUsbPowerMa
    ),
    TUD_MSC_DESCRIPTOR(kUsbItfMsc, 4, kUsbEpOut, kUsbEpIn, kUsbEpSize),
};

// Same as Launcher except entry 3 (serial number), so host logs show "MSC-POC".
const char *kUsbStrings[] = {
    "",
    "M5Stack",
    "Launcher SD",
    "MSC-POC",
    "MSC",
};

void resetUsbDevicePeripheral() {
    REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_IO_MUX_RESET_DISABLE);
    REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
    periph_ll_reset(PERIPH_USB_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_USB_MODULE);
}

void usbTask(void *param) {
    (void)param;
    while (true) {
        tud_task();

        const bool nowMounted = tud_mounted();
        if (nowMounted != s_hostMounted) {
            s_hostMounted = nowMounted;
            appendEvent(nowMounted ? "host mounted (configured)" : "host unmounted");
            if (!nowMounted) flushLog("unmounted"); // host is done with the card, safe to write
        }

        // Only written while the host is not using the export.
        if (!s_hostMounted) {
            const uint32_t now = millis();
            if ((now - s_lastMscActivityMs) >= kIdleFlushMs || (now - s_lastFlushMs) >= kMaxFlushIntervalMs) {
                flushLog("idle");
            }
        }
        vTaskDelay(1);
    }
}

bool beginUsb() {
    if (s_usbStarted) return true;

    // Serial is the HW CDC that owns the USB_SERIAL_JTAG PHY we are about to
    // hand to USB-OTG, so it has to be released first.
    Serial.flush();
    Serial.end();

    resetUsbDevicePeripheral();
    vTaskDelay(pdMS_TO_TICKS(20));

    static const usb_phy_otg_io_conf_t otgIoConf = USB_PHY_SELF_POWERED_DEVICE(-1);
    usb_phy_config_t phyConfig = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
        .ext_io_conf = nullptr,
        .otg_io_conf = &otgIoConf,
    };
    if (usb_new_phy(&phyConfig, &s_usbPhy) != ESP_OK) {
        appendEvent("usb_new_phy failed");
        return false;
    }

    tusb_rhport_init_t tinit = {};
    tinit.role = TUSB_ROLE_DEVICE;
    tinit.speed = TUSB_SPEED_FULL;
    if (!tusb_init(kUsbRhPort, &tinit)) {
        appendEvent("tusb_init failed");
        return false;
    }
    tud_connect();

    xTaskCreate(usbTask, "usb_msc", 4096, nullptr, configMAX_PRIORITIES - 1, &s_usbTask);
    s_usbStarted = true;
    appendEvent("usb started (vid 0x303A pid 0x1001, full speed)");
    return true;
}

void endUsb() {
    if (!s_usbStarted) return;
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(30));
    tusb_deinit(kUsbRhPort);
    if (s_usbTask) {
        vTaskDelete(s_usbTask);
        s_usbTask = nullptr;
    }
    if (s_usbPhy) {
        usb_del_phy(s_usbPhy);
        s_usbPhy = nullptr;
    }
    resetUsbDevicePeripheral();
    s_usbStarted = false;
}

} // namespace

// ---------------------------------------------------------------------------
// TinyUSB descriptors and class callbacks
// ---------------------------------------------------------------------------

extern "C" uint8_t const *tud_descriptor_device_cb(void) {
    return reinterpret_cast<uint8_t const *>(&kDeviceDescriptor);
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return kConfigDescriptor;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc[32];
    uint8_t chrCount = 0;

    if (index == 0) {
        desc[1] = 0x0409;
        chrCount = 1;
    } else {
        if (index >= (sizeof(kUsbStrings) / sizeof(kUsbStrings[0]))) return nullptr;
        const char *str = kUsbStrings[index];
        chrCount = static_cast<uint8_t>(strlen(str) < 31 ? strlen(str) : 31);
        for (uint8_t i = 0; i < chrCount; ++i) desc[1 + i] = str[i];
    }

    desc[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2 * chrCount + 2));
    return desc;
}

// NOTE: tud_mount_cb/tud_umount_cb are defined (strong) by the Arduino core
// (cores/esp32/USB.cpp) whenever CONFIG_TINYUSB_ENABLED is set, so the mount
// state is polled from the usb task instead - same as Launcher does.

extern "C" uint8_t tud_msc_get_maxlun_cb(void) { return 0; }

extern "C" void
tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    memset(vendor_id, ' ', 8);
    memset(product_id, ' ', 16);
    memset(product_rev, ' ', 4);
    memcpy(vendor_id, "M5Stack", 7);
    memcpy(product_id, "Launcher SD", 11);
    memcpy(product_rev, "1.0", 3);
    appendEvent("INQUIRY");
}

extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return true;
}

extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = SD.numSectors();
    *block_size = static_cast<uint16_t>(SD.sectorSize());
}

extern "C" bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun;
    (void)power_condition;
    if (!start && load_eject) {
        appendEvent("eject requested by host -> resetting to leave USB mode");
        flushLog("eject", true); // host said "safe to remove", it is done with the card
        s_ejectRequested = true;
    }
    return true;
}

extern "C" int32_t
tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void)lun;
    noteActivity();
    if (s_readCount == 0) s_readFirstLba = lba;
    s_readLastLba = lba;
    s_readCount++;
    s_readBytes += bufsize;

    const uint32_t secSize = SD.sectorSize();
    if (secSize == 0 || secSize > 512) {
        s_errorCount++;
        appendEvent("RD lba=%lu: bad sector size %lu", (unsigned long)lba, (unsigned long)secSize);
        tud_msc_set_sense(0, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
    }

    MscBlockDevice dev{nullptr, sdReadSector, sdWriteSector, secSize};
    if (!mscTransfer(dev, false, lba, offset, reinterpret_cast<uint8_t *>(buffer), bufsize, s_sectorBuf)) {
        s_errorCount++;
        appendEvent("RD lba=%lu off=%lu size=%lu failed", (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize);
        tud_msc_set_sense(0, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
        return -1;
    }

    return bufsize;
}

extern "C" int32_t
tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    noteActivity();
    if (s_writeCount == 0) s_writeFirstLba = lba;
    s_writeLastLba = lba;
    s_writeCount++;
    s_writeBytes += bufsize;

    const uint32_t secSize = SD.sectorSize();
    if (secSize == 0 || secSize > 512) {
        s_errorCount++;
        appendEvent("WR lba=%lu: bad sector size %lu", (unsigned long)lba, (unsigned long)secSize);
        tud_msc_set_sense(0, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
    }

    MscBlockDevice dev{nullptr, sdReadSector, sdWriteSector, secSize};
    if (!mscTransfer(dev, true, lba, offset, buffer, bufsize, s_sectorBuf)) {
        s_errorCount++;
        appendEvent("WR lba=%lu off=%lu size=%lu failed", (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize);
        tud_msc_set_sense(0, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
        return -1;
    }

    return bufsize;
}

extern "C" bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return true;
}

extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void)buffer;
    (void)bufsize;
    noteActivity();
    s_opCount[scsi_cmd[0]]++;

    switch (scsi_cmd[0]) {
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL: return 0;
        case 0x35: return 0; // SYNCHRONIZE CACHE (10)
        default:
            s_errorCount++;
            appendEvent("scsi_cb: unsupported opcode 0x%02X", scsi_cmd[0]);
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\n%s USB MSC PoC %s\n", DEVICE_NAME, kVersion);

    pinMode(SDCARD_CS, OUTPUT);
    digitalWrite(SDCARD_CS, HIGH);
    delay(250); // let the card's supply settle after power-up / reset

    // A card left mid-command by a previous run can need several attempts, and
    // a slower clock, before it answers CMD0.
    static const uint32_t kSdFreqs[] = {20000000, 10000000, 4000000, 1000000, 400000};
    bool sdOk = false;
    for (int attempt = 0; attempt < 10 && !sdOk; ++attempt) {
        const uint32_t freq = kSdFreqs[attempt % (sizeof(kSdFreqs) / sizeof(kSdFreqs[0]))];
        SD.end();
        s_sdcardSPI.end();
        s_sdcardSPI.begin(SDCARD_SCK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);
        delay(50);
        sdOk = SD.begin(SDCARD_CS, s_sdcardSPI, freq);
        Serial.printf("SD.begin attempt %d @ %lu Hz: %s (type %d)\n", attempt + 1, (unsigned long)freq,
                      sdOk ? "ok" : "fail", sdOk ? (int)SD.cardType() : -1);
    }
    if (!sdOk) {
        while (true) {
            Serial.println("SD mount failed - nothing to export, halting");
            Serial.println("check: card seated, FAT32 formatted, try another card");
            delay(1000);
        }
    }
    Serial.printf("SD: %lu sectors x %lu B\n", (unsigned long)SD.numSectors(), (unsigned long)SD.sectorSize());

    s_logReady = true;
    logHeader();
    s_lastMscActivityMs = millis();
    s_lastFlushMs = millis();

    // The host needs a moment to reattach the CDC port after a reset, so keep
    // reporting status for a while before beginUsb() takes the PHY away.
    for (int i = 2; i > 0; --i) {
        Serial.printf("SD ok, %lu MB; starting USB MSC in %d s\n",
                      (unsigned long)(((uint64_t)SD.numSectors() * SD.sectorSize()) >> 20), i);
        delay(1000);
    }

    if (!beginUsb()) {
        flushLog("usb failed"); // usbTask is not running, so flush here
        Serial.begin(115200);
        while (true) {
            Serial.println("USB MSC start failed, see /msc-poc.log");
            delay(1000);
        }
    }
}

void loop() {
    if (s_ejectRequested) {
        endUsb();
        delay(200);
        ESP.restart();
    }
    delay(50);
}
