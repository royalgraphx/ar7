/*
 * Raspberry Pi emulation (c) 2016 Stefan Weil
 *
 * BCM2835 MMCI0
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef BCM2835_MMCI0_H
#define BCM2835_MMCI0_H

#include "hw/sysbus.h"
#include "exec/address-spaces.h"

#define TYPE_BCM2835_MMCI0 "bcm2835-mmci0"
#define BCM2835_MMCI0(obj) \
        OBJECT_CHECK(BCM2835Mmci0State, (obj), TYPE_BCM2835_MMCI0)

typedef struct {
    /*< private >*/
    SysBusDevice busdev;
    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
} BCM2835Mmci0State;

#endif /* BCM2835_MMCI0_H */
