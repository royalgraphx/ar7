/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 random number generator
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_GPIO_H
#define BCM2835_GPIO_H

#include "hw/sysbus.h"
#include "exec/address-spaces.h"

#define TYPE_BCM2835_GPIO "bcm2835-gpio"
#define BCM2835_GPIO(obj) \
        OBJECT_CHECK(BCM2835GpioState, (obj), TYPE_BCM2835_GPIO)

typedef struct {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t reg[41];
    uint32_t fsel[6];
    uint32_t eds[2];
    uint32_t out[2];
} BCM2835GpioState;

#endif /* BCM2835_GPIO_H */
