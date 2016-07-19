/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 random number generator
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_RNG_H
#define BCM2835_RNG_H

#include "hw/sysbus.h"
#include "exec/address-spaces.h"

#define TYPE_BCM2835_RNG "bcm2835-rng"
#define BCM2835_RNG(obj) \
        OBJECT_CHECK(BCM2835RngState, (obj), TYPE_BCM2835_RNG)

typedef struct {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t ctrl;
    uint32_t status;
    uint32_t data;
} BCM2835RngState;

#endif /* BCM2835_RNG_H */
