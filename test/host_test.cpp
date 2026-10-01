// Host-side test for src/msc_transfer.h.
//
// Verifies the sector/offset maths that macOS uses during mount/probe (the
// failure mode described in Launcher issue #424) against an independent
// byte-offset model of the card, with no hardware involved.
//
// Build/run: test/run.sh

#include "../src/msc_transfer.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool ok, const std::string &what) {
    if (!ok) {
        printf("FAIL: %s\n", what.c_str());
        g_failures++;
    }
}

constexpr uint32_t kSectorSize = 512;
constexpr uint32_t kSectors = 64;

// Fake card: a flat byte image plus per-sector failure injection.
struct FakeCard {
    std::vector<uint8_t> image; // kSectors * kSectorSize
    int failSector = -1;

    FakeCard() : image(kSectors * kSectorSize) {
        for (size_t i = 0; i < image.size(); ++i) image[i] = static_cast<uint8_t>((i * 7 + 13) & 0xFF);
    }

    static bool readSector(void *ctx, uint32_t sector, uint8_t *dst) {
        FakeCard *self = static_cast<FakeCard *>(ctx);
        if (static_cast<int>(sector) == self->failSector) return false;
        if (sector >= kSectors) {
            self->outOfRange = true;
            return false;
        }
        memcpy(dst, self->image.data() + sector * kSectorSize, kSectorSize);
        return true;
    }

    static bool writeSector(void *ctx, uint32_t sector, const uint8_t *src) {
        FakeCard *self = static_cast<FakeCard *>(ctx);
        if (static_cast<int>(sector) == self->failSector) return false;
        if (sector >= kSectors) {
            self->outOfRange = true;
            return false;
        }
        memcpy(self->image.data() + sector * kSectorSize, src, kSectorSize);
        return true;
    }

    bool outOfRange = false;

    MscBlockDevice dev() { return MscBlockDevice{this, readSector, writeSector, kSectorSize}; }
};

// Independent model: absolute byte offsets, no sector arithmetic.
struct Model {
    std::vector<uint8_t> image;
    Model() : image(kSectors * kSectorSize) {
        for (size_t i = 0; i < image.size(); ++i) image[i] = static_cast<uint8_t>((i * 7 + 13) & 0xFF);
    }
    void writeAt(uint32_t lba, uint32_t offset, const uint8_t *src, uint32_t size) {
        memcpy(image.data() + lba * kSectorSize + offset, src, size);
    }
};

uint8_t scratch[kSectorSize];
constexpr size_t kGuard = 32;

// Payload wrapped in canaries, so buffer overruns on either side are caught.
std::vector<uint8_t> makeBuffer(uint32_t size, uint8_t seed) {
    std::vector<uint8_t> buf(size + 2 * kGuard, 0xA5);
    for (uint32_t i = 0; i < size; ++i) buf[kGuard + i] = static_cast<uint8_t>(seed + i);
    return buf;
}

void testRead(FakeCard &card, Model &model, uint32_t lba, uint32_t offset, uint32_t size, const std::string &name) {
    std::vector<uint8_t> got(size + 2 * kGuard, 0xA5);
    uint8_t *payload = got.data() + kGuard;

    MscBlockDevice dev = card.dev();
    const bool ok = mscTransfer(dev, false, lba, offset, payload, size, scratch);

    check(ok, name + ": transfer returned false");
    check(memcmp(payload, model.image.data() + lba * kSectorSize + offset, size) == 0, name + ": wrong bytes");
    for (size_t i = 0; i < kGuard; ++i) {
        check(got[i] == 0xA5, name + ": wrote before buffer start");
        check(got[kGuard + size + i] == 0xA5, name + ": wrote past buffer end");
    }
    check(!card.outOfRange, name + ": accessed a sector outside the card");
}

void testWrite(FakeCard &card, Model &model, uint32_t lba, uint32_t offset, uint32_t size, const std::string &name) {
    std::vector<uint8_t> buf = makeBuffer(size, 0x31);
    uint8_t *payload = buf.data() + kGuard;
    std::vector<uint8_t> sent(payload, payload + size);

    MscBlockDevice dev = card.dev();
    const bool ok = mscTransfer(dev, true, lba, offset, payload, size, scratch);

    check(ok, name + ": transfer returned false");
    if (ok) model.writeAt(lba, offset, sent.data(), size);
    check(card.image == model.image, name + ": card contents differ from the model");
    check(!card.outOfRange, name + ": accessed a sector outside the card");
    for (size_t i = 0; i < kGuard; ++i) {
        check(buf[i] == 0xA5, name + ": write touched the source buffer before its start");
        check(buf[kGuard + size + i] == 0xA5, name + ": write touched the source buffer past its end");
    }
}

} // namespace

int main() {
    // --- reads -------------------------------------------------------------
    {
        FakeCard card;
        Model model;
        testRead(card, model, 3, 0, 512, "read one aligned sector");
        testRead(card, model, 3, 100, 200, "read partial sector");
        testRead(card, model, 3, 400, 300, "read across sector boundary");
        testRead(card, model, 2, 0, 1536, "read 3 aligned sectors");
        testRead(card, model, 1, 100, 1500, "read offset + 3 sectors");
        testRead(card, model, 7, 511, 1, "read last byte of a sector");
    }

    // --- writes ------------------------------------------------------------
    {
        FakeCard card;
        Model model;
        testWrite(card, model, 5, 0, 512, "write one aligned sector");
        testWrite(card, model, 5, 500, 40, "write tail of a sector (rmw)");
        testWrite(card, model, 5, 0, 40, "write head of a sector (rmw)");
        testWrite(card, model, 5, 200, 700, "write across sector boundary");
        testWrite(card, model, 9, 0, 2048, "write 4 aligned sectors");
        testWrite(card, model, 9, 250, 1300, "write offset + multiple sectors");
        testWrite(card, model, 2, 100, 200, "write partial sector only");
    }

    // Byte-preservation on a read-modify-write: everything outside the written
    // range must keep its original value.
    {
        FakeCard card;
        Model model;
        const uint32_t lba = 11, offset = 300, size = 50;
        std::vector<uint8_t> buf = makeBuffer(size, 0x99);
        uint8_t *payload = buf.data() + kGuard;
        MscBlockDevice dev = card.dev();
        check(mscTransfer(dev, true, lba, offset, payload, size, scratch), "rmw: transfer returned false");

        const uint32_t start = lba * kSectorSize;
        check(memcmp(card.image.data() + start, model.image.data() + start, offset) == 0, "rmw: bytes before the write changed");
        check(
            memcmp(card.image.data() + start + offset + size, model.image.data() + start + offset + size, kSectorSize - offset - size) == 0,
            "rmw: bytes after the write changed"
        );
    }

    // --- failure paths -----------------------------------------------------
    {
        FakeCard card;
        uint8_t buf[512];
        card.failSector = 4;
        MscBlockDevice dev = card.dev();
        check(!mscTransfer(dev, false, 4, 0, buf, 512, scratch), "read of a failing sector must fail");
        check(!mscTransfer(dev, true, 4, 0, buf, 512, scratch), "write of a failing sector must fail");
        check(!mscTransfer(dev, false, 3, 400, buf, 300, scratch), "read crossing a failing sector must fail");
        check(mscTransfer(dev, false, 8, 0, buf, 512, scratch), "read of a healthy sector after a failure must succeed");
    }

    // --- geometry guards ---------------------------------------------------
    {
        FakeCard card;
        uint8_t buf[512];
        MscBlockDevice zero = card.dev();
        zero.sectorSize = 0;
        check(!mscTransfer(zero, false, 0, 0, buf, 512, scratch), "sector size 0 must be rejected");

        MscBlockDevice big = card.dev();
        big.sectorSize = 1024;
        check(!mscTransfer(big, false, 0, 0, buf, 512, scratch), "sector size 1024 must be rejected");

        MscBlockDevice ok = card.dev();
        check(!mscTransfer(ok, false, 0, 512, buf, 512, scratch), "offset beyond the sector size must be rejected");
        check(mscTransfer(ok, false, 0, 0, buf, 0, scratch), "zero-length transfer must succeed");
    }

    if (g_failures == 0) {
        printf("ok\n");
        return 0;
    }
    printf("%d failure(s)\n", g_failures);
    return 1;
}
