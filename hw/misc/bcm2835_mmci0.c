/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 random number generator
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
Unassigned mem write 000000002020004c = 0x0 [][]
Unassigned mem write 0000000020200058 = 0x0 [][]
Unassigned mem write 0000000020200064 = 0x0 [][]
Unassigned mem write 0000000020200070 = 0x0 [][]
Unassigned mem write 000000002020007c = 0x0 [][]
Unassigned mem write 0000000020200088 = 0x0 [][]
Unassigned mem read 0000000020200040 [][]
Unassigned mem write 0000000020200050 = 0x0 [][]
Unassigned mem write 000000002020005c = 0x0 [][]
Unassigned mem write 0000000020200068 = 0x0 [][]
Unassigned mem write 0000000020200074 = 0x0 [][]
Unassigned mem write 0000000020200080 = 0x0 [][]
Unassigned mem write 000000002020008c = 0x0 [][]
Unassigned mem read 0000000020200044 [][]
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/bcm2835_mmci0.h"

static uint64_t bcm2835_mmci0_read(void *opaque, hwaddr offset, unsigned size)
{
    //~ BCM2835Mmci0State *s = (BCM2835Mmci0State *)opaque;
    uint32_t res = 0;

    switch (offset) {
    case 0x00:
    case 0x10:
    case 0x20:
    case 0x34:
        qemu_log_mask(LOG_UNIMP,
                      "%s: Unimplemented offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Unimplemented offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
    }
    return res;
}

static void bcm2835_mmci0_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    //~ BCM2835Mmci0State *s = (BCM2835Mmci0State *)opaque;
    switch (offset) {
    case 0x00:
    case 0x04:
    case 0x08:
    case 0x0c:
    case 0x20:
    case 0x30:
    case 0x34:
    case 0x38:
    case 0x3c:
    case 0x50:
        qemu_log_mask(LOG_UNIMP,
                      "%s: Unimplemented offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
    }
}

static const MemoryRegionOps bcm2835_mmci0_ops = {
    .read = bcm2835_mmci0_read,
    .write = bcm2835_mmci0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_mmci0 = {
    .name = TYPE_BCM2835_MMCI0,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_mmci0_init(Object *obj)
{
    BCM2835Mmci0State *s = BCM2835_MMCI0(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    memory_region_init_io(&s->iomem, obj, &bcm2835_mmci0_ops, s,
                          TYPE_BCM2835_MMCI0, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void bcm2835_mmci0_realize(DeviceState *dev, Error **errp)
{
    //~ BCM2835Mmci0State *s = BCM2835_MMCI0(dev);
}

static void bcm2835_mmci0_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_mmci0_realize;
    dc->vmsd = &vmstate_bcm2835_mmci0;
}

static TypeInfo bcm2835_mmci0_info = {
    .name          = TYPE_BCM2835_MMCI0,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835Mmci0State),
    .class_init    = bcm2835_mmci0_class_init,
    .instance_init = bcm2835_mmci0_init,
};

static void bcm2835_mmci0_register_types(void)
{
    type_register_static(&bcm2835_mmci0_info);
}

type_init(bcm2835_mmci0_register_types)
