/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 GPIO
 *
 * See BCM2835 ARM Peripherals:
 * https://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/bcm2835_gpio.h"

static uint64_t bcm2835_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2835GpioState *s = opaque;
    uint32_t res = 0;

    switch (offset) {
    //~ case 0x00: /* GPIO Function Select 0 */
    case 0x04: /* GPIO Function Select 1 */
    //~ case 0x08: /* GPIO Function Select 2 */
    //~ case 0x0c: /* GPIO Function Select 3 */
    case 0x10: /* GPIO Function Select 4 */
    case 0x14: /* GPIO Function Select 5 */
        res = s->fsel[offset / 4];
        break;
    case 0x34: /* GPIO Pin Level 0 */
        res = s->out[0];
        break;
    //~ case 0x38: /* GPIO Pin Level 1 */
    case 0x40: /* GPIO Pin Event Detect Status 0 */
    case 0x44: /* GPIO Pin Event Detect Status 1 */
        assert(offset / 4 < ARRAY_SIZE(s->reg));
        res = s->reg[offset / 4];
        qemu_log_mask(LOG_UNIMP,
                      "%s: Unimplemented offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%02" HWADDR_PRIx ", size %u\n",
                      __func__, offset, size);
    }
    return res;
}

static void bcm2835_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    BCM2835GpioState *s = opaque;
    switch (offset) {
    //~ case 0x00: /* GPIO Function Select 0 */
    case 0x04: /* GPIO Function Select 1 */
    //~ case 0x08: /* GPIO Function Select 2 */
    //~ case 0x0c: /* GPIO Function Select 3 */
    case 0x10: /* GPIO Function Select 4 */
    case 0x14: /* GPIO Function Select 5 */
        s->fsel[offset / 4] = value;
        break;
    case 0x1c: /* GPIO Pin Output Set 0 */
        s->out[0] |= value;
        break;
    case 0x20: /* GPIO Pin Output Set 1 */
        s->out[1] |= value;
        break;
    case 0x28: /* GPIO Pin Output Clear 0 */
        s->out[0] &= ~value;
        break;
    case 0x2c: /* GPIO Pin Output Clear 1 */
        s->out[1] &= ~value;
        break;
    case 0x4c: /* GPIO Pin Rising Edge Detect Enable 0 */
    case 0x50: /* GPIO Pin Rising Edge Detect Enable 0 */
    case 0x58: /* GPIO Pin Falling Edge Detect Enable 0 */
    case 0x5c: /* GPIO Pin Falling Edge Detect Enable 1 */
    case 0x64: /* GPIO Pin High Detect Enable 0 */
    case 0x68: /* GPIO Pin High Detect Enable 1 */
    case 0x70: /* GPIO Pin Low Detect Enable 0 */
    case 0x74: /* GPIO Pin Low Detect Enable 1 */
    case 0x7c: /* GPIO Pin Async. Rising Edge Detect 0 */
    case 0x80: /* GPIO Pin Async. Rising Edge Detect 1 */
    case 0x88: /* GPIO Pin Async. Falling Edge Detect 0 */
    case 0x8c: /* GPIO Pin Async. Falling Edge Detect 1 */
        assert(offset / 4 < ARRAY_SIZE(s->reg));
        s->reg[offset / 4] = value;
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

static const MemoryRegionOps bcm2835_gpio_ops = {
    .read = bcm2835_gpio_read,
    .write = bcm2835_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_gpio = {
    .name = TYPE_BCM2835_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2835_gpio_init(Object *obj)
{
    BCM2835GpioState *s = BCM2835_GPIO(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    memory_region_init_io(&s->iomem, obj, &bcm2835_gpio_ops, s,
                          TYPE_BCM2835_GPIO, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void bcm2835_gpio_realize(DeviceState *dev, Error **errp)
{
    BCM2835GpioState *s = BCM2835_GPIO(dev);
    memset(s->reg, 0, sizeof(s->reg));
    s->out[0] = 0;
    s->out[1] = 0;
}

static void bcm2835_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bcm2835_gpio_realize;
    dc->vmsd = &vmstate_bcm2835_gpio;
}

static TypeInfo bcm2835_gpio_info = {
    .name          = TYPE_BCM2835_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2835GpioState),
    .class_init    = bcm2835_gpio_class_init,
    .instance_init = bcm2835_gpio_init,
};

static void bcm2835_gpio_register_types(void)
{
    type_register_static(&bcm2835_gpio_info);
}

type_init(bcm2835_gpio_register_types)
