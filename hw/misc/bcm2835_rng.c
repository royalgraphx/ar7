/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 random number generator
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/bcm2835_rng.h"


#define RNG_CTRL        0x0
#define RNG_STATUS      0x4
#define RNG_DATA        0x8

/* Enable rng. */
#define RNG_RBGEN       0x1

/* the initial numbers generated are "less random" so will be discarded */
#define RNG_WARMUP_COUNT 0x40000


static uint64_t bcm2835_rng_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2835RngState *s = (BCM2835RngState *)opaque;
    uint32_t res = 0;

    switch (offset) {
    case RNG_STATUS:
        s->status ^= 1U << 24;
        res = s->status;
        break;
    case RNG_DATA:
        res = s->data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
    }
    return res;
}

static void bcm2835_rng_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    BCM2835RngState *s = (BCM2835RngState *)opaque;
    switch (offset) {
    case RNG_CTRL:
        s->ctrl = value;
        break;
    case RNG_STATUS:
        s->status = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
    }
}

static const MemoryRegionOps bcm2835_rng_ops = {
    .read = bcm2835_rng_read,
    .write = bcm2835_rng_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_rng = {
    .name = TYPE_BCM2835_RNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_rng_init(Object *obj)
{
    BCM2835RngState *s = BCM2835_RNG(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    memory_region_init_io(&s->iomem, obj, &bcm2835_rng_ops, s,
                          TYPE_BCM2835_RNG, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void bcm2835_rng_realize(DeviceState *dev, Error **errp)
{
    BCM2835RngState *s = BCM2835_RNG(dev);
    s->ctrl = 0;
    s->status = 0;
    s->data = 0;
}

static void bcm2835_rng_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_rng_realize;
    dc->vmsd = &vmstate_bcm2835_rng;
}

static TypeInfo bcm2835_rng_info = {
    .name          = TYPE_BCM2835_RNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835RngState),
    .class_init    = bcm2835_rng_class_init,
    .instance_init = bcm2835_rng_init,
};

static void bcm2835_rng_register_types(void)
{
    type_register_static(&bcm2835_rng_info);
}

type_init(bcm2835_rng_register_types)
