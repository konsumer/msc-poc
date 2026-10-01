// Sector-transfer core of the USB MSC read/write callbacks.
//
// Dependency-free on purpose: the firmware compiles it against SdFat, the host
// test (test/host_test.cpp) compiles it against a fake card, so the
// offset/partial-sector maths that macOS exercises during mount is verifiable
// without hardware.
//
// This is the logic behind Launcher PR #424: a host sends READ10/WRITE10 with a
// non-zero byte offset and lengths that do not line up with sector boundaries,
// and the device has to satisfy them instead of failing the transfer.

#pragma once

#include <stdint.h>
#include <string.h>

struct MscBlockDevice {
    void *ctx;
    bool (*readSector)(void *ctx, uint32_t sector, uint8_t *dst);
    bool (*writeSector)(void *ctx, uint32_t sector, const uint8_t *src);
    uint32_t sectorSize;
};

// Transfers `size` bytes between `buffer` and the sector device, starting at
// byte `offset` inside sector `lba`. `scratch` must hold one sector.
// Returns false if the device rejects a sector, or if the geometry is unusable
// (sector size 0, above 512, or a start offset outside the first sector).
inline bool mscTransfer(
    MscBlockDevice &dev, bool write, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t size, uint8_t *scratch
) {
    const uint32_t secSize = dev.sectorSize;
    if (secSize == 0 || secSize > 512) return false;
    if (offset >= secSize) return false;
    if (size == 0) return true;

    uint32_t sector = lba + (offset / secSize);
    uint32_t sectorOffset = offset % secSize;
    uint32_t remaining = size;
    uint8_t *ptr = buffer;

    while (remaining > 0) {
        const uint32_t chunk = remaining < (secSize - sectorOffset) ? remaining : (secSize - sectorOffset);

        if (sectorOffset == 0 && chunk == secSize) {
            // Whole sector: straight transfer, no scratch buffer needed.
            const bool ok = write ? dev.writeSector(dev.ctx, sector, ptr) : dev.readSector(dev.ctx, sector, ptr);
            if (!ok) return false;
        } else if (write) {
            // Partial sector: read-modify-write so the untouched bytes survive.
            if (!dev.readSector(dev.ctx, sector, scratch)) return false;
            memcpy(scratch + sectorOffset, ptr, chunk);
            if (!dev.writeSector(dev.ctx, sector, scratch)) return false;
        } else {
            if (!dev.readSector(dev.ctx, sector, scratch)) return false;
            memcpy(ptr, scratch + sectorOffset, chunk);
        }

        remaining -= chunk;
        ptr += chunk;
        sector++;
        sectorOffset = 0;
    }

    return true;
}
