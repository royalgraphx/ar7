/*
 * ARM Raspberry Pi System emulation.
 *
 * Copyright (c) 2012 Stefan Weil
 *
 * This code is licensed under the GNU GPL 2 or later.
 *
 * This very basic emulation of Broadcom's BCM2835 media processor
 * works (partially as of 2012-07) with the Linux BCM2708 code.
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0183g/DDI0183G_uart_pl011_r1p5_trm.pdf
 * http://elinux.org/RPi_Framebuffer
 * https://github.com/raspberrypi/firmware/wiki/Mailboxes
 *
 */

#include "qemu-common.h"
#include "blockdev.h"
#include "console.h"            /* graphic_console_init */
#include "exec-memory.h"
#include "hw/arm-misc.h"
#include "hw/boards.h"
#include "hw/devices.h"
#include "hw/flash.h"
#include "hw/framebuffer.h"     /* framebuffer_update_display */
#include "hw/sd.h"              /* sd_init, ... */
#include "hw/sysbus.h"
//~ #include "i2c.h"
#include "net.h"
#include "raspi.h"
#include "sysemu.h"

#define logout(fmt, ...) \
    fprintf(stderr, "RPI\t%-24s" fmt, __func__, ##__VA_ARGS__)

static const char *bt(void)
{
    static char bt_buffer[256];
    return qemu_sprint_backtrace(bt_buffer, sizeof(bt_buffer));
}

#define BCM2708
#include "hw/pl011.c"

#define IO_SIZE (16 * MiB)
#define RAM_SIZE (256 * MiB)

#define SZ_4K   4096

typedef struct {
    MemoryRegion iomem;
    qemu_irq irq[MAXIRQNUM];
    uint32_t irq_basic_pending;
    uint32_t irq_pending_1;
    uint32_t irq_pending_2;
    uint32_t fiq_control;
    uint32_t enable_irqs_1;
    uint32_t enable_irqs_2;
    uint32_t enable_basic_irqs;
} BCM2708InterruptController;

typedef struct {
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint64_t clock;     // initial value  of QEMU timer
    int64_t expire;
    uint32_t dt;
    uint32_t cs;
    uint32_t c[4];
    bool active[4];
} BCM2708SystemTimer;

typedef struct {
    uint32_t xres, yres, xres_virtual, yres_virtual;
    uint32_t pitch, bpp;
    uint32_t xoffset, yoffset;
    uint32_t base;
    uint32_t screen_size;
    uint16_t cmap[256];
} FBInfo;

typedef struct {
    DisplayState *ds;
    drawfn *line_fn;
    unsigned cols;
    unsigned rows;
    unsigned width;
    unsigned src_width;
    unsigned dest_width;
    bool enable;
    bool need_update;
    FBInfo info;
} BCM2708Framebuffer;

typedef struct {
    SDState *card;
    bool enabled;
} BCM2708SDCard;

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    MemoryRegion alias_4;
    MemoryRegion alias_8;
    MemoryRegion alias_c;
    BCM2708InterruptController ic;      /* interrupt controller */
    BCM2708SystemTimer st;              /* system timer */
    BCM2708Framebuffer fb;              /* frame buffer */
    BCM2708SDCard emmc;                 /* external mass media controller */
    pl011_state uart0;
    //~ uint32_t level;
    //~ uint32_t mask;
    //~ int irq;
} BCM2708State;

typedef struct {
    MemoryRegion ram;
    MemoryRegion ram_4_alias;
    MemoryRegion ram_8_alias;
    MemoryRegion ram_c_alias;
    struct arm_boot_info binfo;
    BCM2708State *bcm2708;
    qemu_irq *cpu_pic;
} RaspberryPi;

static RaspberryPi *rpi;

static void bcm2708_update_irq(void)
{
    BCM2708InterruptController *ic = &rpi->bcm2708->ic;
    bool pic_irq = (ic->irq_basic_pending & ic->enable_basic_irqs) ||
                   (ic->irq_pending_1 & ic->enable_irqs_1) ||
                   (ic->irq_pending_2 & ic->enable_irqs_2);
    //~ logout("pic irq = %u\n", pic_irq);
    qemu_set_irq(rpi->cpu_pic[ARM_PIC_CPU_IRQ], pic_irq);
}

static void bcm2708_set_irq(unsigned num, bool value)
{
    if (num == INTERRUPT_ARM_MAILBOX) {
        logout("interrupt %u = %u\n", num, value);
    }

    switch (num) {
    case INTERRUPT_TIMER0 ... INTERRUPT_VPUDMA:
        if (value) {
            rpi->bcm2708->ic.irq_pending_1 |=
                (1 << (num - INTERRUPT_TIMER0));
        } else {
            rpi->bcm2708->ic.irq_pending_1 &=
                ~(1 << (num - INTERRUPT_TIMER0));
        }
        if (rpi->bcm2708->ic.irq_pending_1) {
            rpi->bcm2708->ic.irq_basic_pending |= (1 << 8);
        } else {
            rpi->bcm2708->ic.irq_basic_pending &= ~(1 << 8);
        }
        bcm2708_update_irq();
        break;
    case INTERRUPT_ARM_TIMER ... INTERRUPT_ARASANSDIO:
        if (value) {
            rpi->bcm2708->ic.irq_basic_pending |=
                (1 << (num - INTERRUPT_ARM_TIMER));
        } else {
            rpi->bcm2708->ic.irq_basic_pending &=
                ~(1 << (num - INTERRUPT_ARM_TIMER));
        }
        bcm2708_update_irq();
        break;
    default:
        hw_error("unexpected interrupt %u\n", num);
    }
}

static uint64_t bcm2708_timer_clock(BCM2708SystemTimer *st)
{
    return qemu_get_clock_us(vm_clock) - st->clock;
}

static void bcm2708_timer_update(BCM2708SystemTimer *st)
{
    uint64_t now = qemu_get_clock_us(vm_clock);
    uint32_t clo = (uint32_t)(now - st->clock);
    uint32_t dt_min = UINT32_MAX;
    uint8_t i;
    for (i = 0; i < 4; i++) {
        uint32_t dt = st->c[i] - clo;
        if (dt != 0 && dt < dt_min) {
            dt_min = dt;
        }
    }
    for (i = 0; i < 4; i++) {
        uint32_t dt = st->c[i] - clo;
        st->active[i] = (dt == dt_min);
    }
    st->dt = dt_min;
    st->expire = now + dt_min;
    //~ logout("wait %" PRIu32 " µs\n", dt_min);
    qemu_mod_timer(st->timer, st->expire);
}

static void bcm2708_timer_irq(BCM2708SystemTimer *st)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        if (st->cs & (1 << i)) {
            bcm2708_set_irq(INTERRUPT_TIMER0 + i, true);
        } else {
            bcm2708_set_irq(INTERRUPT_TIMER0 + i, false);
        }
    }
}

static void bcm2708_timer_tick(void *opaque)
{
    BCM2708SystemTimer *st = opaque;
    //~ uint64_t now = qemu_get_clock_us(vm_clock);
    //~ uint32_t clo = (uint32_t)(now - st->clock);
    uint8_t cs = st->cs;
    uint8_t i;
    //~ logout("%u µs expired\n", st->dt);
    for (i = 0; i < 4; i++) {
        if (st->active[i] && (cs ^ (1 << i))) {
            st->cs |= (1 << i);
            //~ logout("raise interrupt for C%u\n", i);
            bcm2708_set_irq(INTERRUPT_TIMER0 + i, true);
        } else {
            //~ qemu_set_irq(s->parent_irq, false);
        }
    }
    bcm2708_timer_update(st);
}

/* DMA. */

static uint32_t bcm2708_dma_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO)\n", offset, value);
        return value;
    }
    logout("offset=0x%02x, value=0x%08x\n", offset, value);
    return value;
}

static void bcm2708_dma_write(BCM2708State *s, unsigned offset, uint32_t value)
{
    //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
    //~        offset, value);
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO)\n",
               offset, value);
    }
}

/* ARM Interrupt controller. */

static uint64_t bcm2708_ic_read(void *opaque, target_phys_addr_t offset,
                                unsigned size)
{
    BCM2708InterruptController *ic = opaque;
    uint32_t value = 0;
    assert(size == 4);
    switch (offset) {
    case 0x00:  /* IRQ basic pending */
        value = ic->irq_basic_pending;
        //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
               //~ offset, value);
        break;
    case 0x04: /* IRQ pending 1 */
        value = ic->irq_pending_1;
        //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
               //~ offset, value);
        break;
    case 0x08: /* IRQ pending 2 */
        value = ic->irq_pending_2;
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
               offset, value);
        break;
    case 0x0c: /* FIQ control */
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x0000 (TODO)\n",
               offset);
    }
    return value;
}

static void bcm2708_ic_write(void *opaque, target_phys_addr_t offset, uint64_t value,
                             unsigned size)
{
    BCM2708InterruptController *ic = opaque;

    assert(size == 4);
    switch (offset) {
    case 0x00: /* IRQ basic pending */
    case 0x04: /* IRQ pending 1 */
    case 0x08: /* IRQ pending 2 */
    case 0x0c: /* FIQ control */
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (TODO)\n", offset, value);
        break;
    case 0x10: /* Enable IRQs 1 */
        ic->enable_irqs_1 |= value;
        if (value != 8)
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (Enable IRQs 1)\n", offset, value);
        break;
    case 0x14: /* Enable IRQs 2 */
        ic->enable_irqs_2 |= value;
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (Enable IRQs 2)\n", offset, value);
        break;
    case 0x18: /* Enable Basic IRQs */
        ic->enable_basic_irqs |= value;
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (Enable Basic IRQs)\n", offset, value);
        break;
    case 0x1c: /* Disable IRQs 1 */
        //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (disable)\n", offset, value);
        ic->enable_irqs_1 &= ~value;
        bcm2708_update_irq();
        break;
    case 0x20: /* Disable IRQs 2 */
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (Disable IRQs 2)\n", offset, value);
        ic->enable_irqs_2 &= ~value;
        bcm2708_update_irq();
        break;
    case 0x24: /* Disable Basic IRQs */
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (Disable Basic IRQs)\n", offset, value);
        ic->enable_basic_irqs &= ~value;
        bcm2708_update_irq();
        break;
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (TODO)\n", offset, value);
    }
}

static uint64_t bcm2708_armctrl_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
    BCM2708State *s = opaque;
    uint32_t value = 0;

    assert(size == 4);

    if (offset < 0x200) {
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x0000 (TODO)\n",
               offset);
    } else {
        value = bcm2708_ic_read(&s->ic, offset - 0x0200, size);
    }

    return value;
}

static void bcm2708_armctrl_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
    BCM2708State *s = opaque;

    assert(size == 4);

    if (offset < 0x0200) {
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (TODO)\n", offset, value);
    } else {
        bcm2708_ic_write(&s->ic, offset - 0x0200, value, size);
    }
}

/* System timer. */

static uint64_t bcm2708_st_read(void *opaque, target_phys_addr_t offset,
                                unsigned size)
{
    BCM2708SystemTimer *st = opaque;
    uint32_t value = 0;
    assert(size == 4);
    switch (offset) {
    case 0x04: /* CLO */
        value = (uint32_t)bcm2708_timer_clock(st);
        break;
    case 0x08: /* CHI */
        value = (uint32_t)(bcm2708_timer_clock(st) >> 32);
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n", offset, value);
        break;
    case 0x0c ... 0x18: /* C0, C1, C2, C3 */
        value = st->c[(offset - 0x0c) / 4];
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n", offset, value);
        break;
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x (TODO)\n",
               offset, value);
        return value;
    }
    return value;
}

static void bcm2708_st_write(void *opaque, target_phys_addr_t offset,
                             uint64_t value, unsigned size)
{
    BCM2708SystemTimer *st = opaque;
    unsigned timer_index;

    assert(size == 4);

    switch (offset) {
    case 0x00: /* CS */
        //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (CS)\n",
               //~ offset, value);
        st->cs &= ~value;
        bcm2708_timer_irq(st);
        break;
    case 0x04: /* CLO */
    case 0x08: /* CHI */
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (ignored)\n",
               offset, value);
        break;
    case 0x0c ... 0x18: /* C0, C1, C2, C3 */
        timer_index = (offset - 0x0c) / 4;
        st->c[timer_index] = value;
        //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (C%u)\n",
               //~ offset, value, timer_index);
        bcm2708_timer_update(st);
        break;
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (TODO)\n", offset, value);
    }
}

/* UART 0. */

static uint32_t bcm2708_uart0_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
        return value;
    }
    //~ logout("offset=0x%02x, value=0x%08x\n", offset, value);
    return value;
}

/* External Mass Media Controller (eMMC). */

static uint32_t bcm2708_emmc_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    case 0x00:  /* ACMD23 argument */
    case 0x04:  /* block size and count */
    case 0x08:  /* argument */
    case 0x0c:  /* command and transfer mode */
    case 0x28:  /* host configuration bits */
    case 0x2c:  /* host configuration bits */
    case 0x34:  /* interrupt flag enable */
    case 0xfc:  /* slot interrupt status and version */
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
        return value;
    }
    logout("offset=0x%02x, value=0x%08x\n", offset, value);
    return value;
}

static void bcm2708_emmc_write(BCM2708State *s, unsigned offset, uint32_t value)
{
    //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
    //~        offset, value);
    switch (offset) {
    case 0x28:  /* host configuration bits */
    case 0x2c:  /* host configuration bits */
    case 0x34:  /* interrupt flag enable */
    case 0x38:  /* interrupt generation enable */
    case 0x80:  /* expansion fifo configuration */
    case 0x84:  /* expansion fifo enable */
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
    }
}

/* GPIO. */

static uint32_t bcm2708_gpio_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
        return value;
    }
    return value;
}

static void bcm2708_gpio_write(BCM2708State *s, unsigned offset, uint32_t value)
{
    //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
    //~        offset, value);
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
    }
}

/* USB. */

static uint32_t bcm2708_usb_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
        return value;
    }
    logout("offset=0x%02x, value=0x%08x\n", offset, value);
    return value;
}

static void bcm2708_usb_write(BCM2708State *s, unsigned offset, uint32_t value)
{
    //~ logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08x\n",
    //~        offset, value);
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
    }
}

static bool mailbox_empty;
static unsigned mailbox_value;
//~ static uint8_t mailbox[] = { 1 };
//~ static uint8_t mailbox_index;
#define MAILBOX_SIZE ARRAY_SIZE(mailbox)

static uint32_t bcm2708_0_sbm_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    case 0x80:          // ARM_0_MAIL0_RD
        if (!mailbox_empty) {
            value = mailbox_value;
            bcm2708_set_irq(INTERRUPT_ARM_MAILBOX, false);
            mailbox_empty = true;
        }
        logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL0_RD)\n", offset, value);
        break;
    case 0x98:          // ARM_0_MAIL0_STA Status read
        if (mailbox_empty) {
            value |= ARM_MS_EMPTY;
        }
        // TODO: Missing emulation of ARM_MS_FULL for mailbox writes.
        logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL0_STA)\n", offset, value);
        break;
    case 0x9c:          // ARM_0_MAIL0_CNF Config
    case 0xa0:          // ARM_0_MAIL1_WRT
    default:
        logout("offset=0x%02x, value=0x%08x (TODO) %s\n", offset, value,
               bt());
        return value;
    }
    //~ logout("offset=0x%02x, value=0x%08x\n", offset, value);
    return value;
}

//~ Mailbox Peek  Read  Write  Status  Sender  Config
//~    0    0x10  0x00  0x20   0x18    0x14    0x1C
//~    1    0x20

static void bcm2708_0_sbm_write(BCM2708State *s, unsigned offset,
                                uint32_t value)
{
    switch (offset) {
    case 0x9c:          // Config
        if (value == ARM_MC_IHAVEDATAIRQEN) {
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL0_CNF)\n", offset, value);
            //~ bcm2708_set_irq(INTERRUPT_ARM_MAILBOX, true);
        } else {
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL0_CNF, TODO)\n", offset, value);
        }
        /* TODO: Clear error bits. */
        break;
    case 0xa0:          // ARM_0_MAIL1_WRT
        mailbox_empty = false;
        switch (value & 7) {
        case MBOX_CHAN_POWER:
            //~ mailbox_value = 0x4711;
            mailbox_value = 0x00000000 + MBOX_CHAN_POWER;
            if (value == 0x80) {
                mailbox_value = 0x00000080 + MBOX_CHAN_POWER;
            }
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL1_WRT Power)\n", offset, value);
            break;
        case MBOX_CHAN_FB:
            /* Framebuffer read. */
            mailbox_value = 0x00000000 + MBOX_CHAN_FB;
            {
                //~ $9 = (volatile struct fbinfo_s *) 0xffdff000
                //~ (gdb) p *fbinfo
                //~ $10 = {xres = 800, yres = 480, xres_virtual = 800, yres_virtual = 480, pitch = 0, bpp = 16, xoffset = 0, yoffset = 0,
                //~ base = 0, screen_size = 0, cmap = {0 <repeats 256 times>}}
                FBInfo *fbinfo = &s->fb.info;
                unsigned bytes_per_pixel;
                uint32_t addr = value & ~7;
                cpu_physical_memory_read(addr, fbinfo, sizeof(*fbinfo));
                bytes_per_pixel = fbinfo->bpp / 8;
                /* TODO: Do we have to handle 15 bpp or other odd values? */
                assert(bytes_per_pixel * 8 == fbinfo->bpp);
                fbinfo->pitch = fbinfo->xres_virtual * bytes_per_pixel;
                fbinfo->base = rpi->binfo.ram_size;
                fbinfo->screen_size =
                    fbinfo->xres_virtual * fbinfo->yres_virtual * bytes_per_pixel;
                logout("framebuffer %u x %u x %u, %u byte at 0x%08x\n",
                       fbinfo->xres, fbinfo->yres, fbinfo->bpp,
                       fbinfo->screen_size, fbinfo->base);
                cpu_physical_memory_write(addr, fbinfo, sizeof(*fbinfo));

            }
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL1_WRT Framebuffer)\n", offset, value);
            break;
        case MBOX_CHAN_VCHIQ:
            mailbox_value = 0x00000080 + MBOX_CHAN_VCHIQ;
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL1_WRT VCHIQ)\n", offset, value);
            break;
        default:
            mailbox_value = 0x0815;
            logout("offset=0x%02x, value=0x%08x (ARM_0_MAIL1_WRT TODO)\n", offset, value);
        }
        bcm2708_set_irq(INTERRUPT_ARM_MAILBOX, true);
        break;
    default:
        logout("offset=0x%02x, value=0x%08x (TODO)\n", offset, value);
    }
}

#define IO(offset) (offset + BCM2708_PERI_BASE)

static uint64_t bcm2708_read(void *opaque, target_phys_addr_t offset,
                             unsigned size)
{
    BCM2708State *s = opaque;
    uint32_t value = 0;

    assert(size == 4);

    switch (IO(offset)) {
    case DMA_BASE ... DMA_BASE + SZ_4K - 1:
        value = bcm2708_dma_read(s, IO(offset) - DMA_BASE);
        break;
    case UART0_BASE ... UART0_BASE + 0xffc:
        value = bcm2708_uart0_read(s, IO(offset) - UART0_BASE);
        break;
    case ARMCTRL_0_SBM_BASE ... ARMCTRL_0_SBM_BASE + 0xa0:
        value = bcm2708_0_sbm_read(s, IO(offset) - ARMCTRL_0_SBM_BASE);
        break;
    case GPIO_BASE ... GPIO_BASE + SZ_4K - 1:
        value = bcm2708_gpio_read(s, IO(offset) - GPIO_BASE);
        break;
    case EMMC_BASE ... EMMC_BASE + 0xff:
        value = bcm2708_emmc_read(s, IO(offset) - EMMC_BASE);
        break;
    case USB_BASE ... USB_BASE + 0x1ffff:
        value = bcm2708_usb_read(s, IO(offset) - USB_BASE);
        break;
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x0000 (TODO)\n", offset);
    }
    return value;
}

static void bcm2708_write(void *opaque, target_phys_addr_t offset,
                          uint64_t value, unsigned size)
{
    BCM2708State *s = opaque;

    assert(size == 4);

    switch (IO(offset)) {
    case DMA_BASE ... DMA_BASE + SZ_4K - 1:
        bcm2708_dma_write(s, IO(offset) - DMA_BASE, value);
        break;
    case ARMCTRL_0_SBM_BASE ... ARMCTRL_0_SBM_BASE + 0xa0:
        bcm2708_0_sbm_write(s, IO(offset) - ARMCTRL_0_SBM_BASE, value);
        break;
    case GPIO_BASE ... GPIO_BASE + SZ_4K - 1:
        bcm2708_gpio_write(s, IO(offset) - GPIO_BASE, value);
        break;
    case EMMC_BASE ... EMMC_BASE + 0xff:
        bcm2708_emmc_write(s, IO(offset) - EMMC_BASE, value);
        break;
    case USB_BASE ... USB_BASE + 0x1ffff:
        bcm2708_usb_write(s, IO(offset) - USB_BASE, value);
        break;
    default:
        logout("offset=0x%02" TARGET_PRIxPHYS ", value=0x%08" PRIx64 " (TODO)\n", offset, value);
    }
    //~ bcm2708_update(s);
}

static const MemoryRegionOps bcm2708_armctrl_ops = {
    .read = bcm2708_armctrl_read,
    .write = bcm2708_armctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps bcm2708_st_ops = {
    .read = bcm2708_st_read,
    .write = bcm2708_st_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps bcm2708_ops = {
    .read = bcm2708_read,
    .write = bcm2708_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Framebuffer. */

static inline
uint32_t s3c24xx_rgb_to_pixel8(unsigned int r, unsigned int g, unsigned b)
{
    return ((r >> 5) << 5) | ((g >> 5) << 2) | (b >> 6);
}

static inline
uint32_t s3c24xx_rgb_to_pixel15(unsigned int r, unsigned int g, unsigned b)
{
    return ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
}

static inline
uint32_t s3c24xx_rgb_to_pixel16(unsigned int r, unsigned int g, unsigned b)
{
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

static inline
uint32_t s3c24xx_rgb_to_pixel24(unsigned int r, unsigned int g, unsigned b)
{
    return (r << 16) | (g << 8) | b;
}

static inline
uint32_t s3c24xx_rgb_to_pixel32(unsigned int r, unsigned int g, unsigned b)
{
    return (r << 16) | (g << 8) | b;
}

#define BITS 8
#include "hw/s3c24xx_template.h"
#define BITS 15
#include "hw/s3c24xx_template.h"
#define BITS 16
#include "hw/s3c24xx_template.h"
#define BITS 24
#include "hw/s3c24xx_template.h"
#define BITS 32
#include "hw/s3c24xx_template.h"

static void bcm2708_fb_invalidate(void *opaque)
{
    //~ BCM2708State *s = opaque;
    logout("\n");
}

static void bcm2708_fb_dump(void *opaque, const char *filename, bool cswitch,
                            Error **errp)
{
    //~ BCM2708State *s = opaque;
    logout("\n");
}

static void bcm2708_fb_text_update(void *opaque, console_ch_t *chardata)
{
    //~ BCM2708State *s = opaque;
    logout("\n");
}

static void bcm2708_fb_update(void *opaque)
{
#if 0
    /* This function is called frequently. */
    BCM2708Framebuffer *s = opaque;
    int src_width, dest_width, miny = 0, maxy = 0;
    //~ logout("\n");
    if (!s->enable || !s->dest_width)
        return;

    //~ s3c24xx_lcd_resize(s);

    //~ if (s->invalidatep) {
        //~ s3c24xx_lcd_palette_load(s);
        //~ s->invalidatep = 0;
    //~ }

    src_width = s->src_width;
    dest_width = s->width * s->dest_width;

    framebuffer_update_display(s->ds, sysbus_address_space(&s->busdev),
                               s->base, s->cols, s->rows,
                               src_width, dest_width, 0,
                               s->need_update,
                               s->fn, s->palette,
                               &miny, &maxy);
#endif
}

/* -------------------------------------------------------------------------- */

//~ static BCM2708State *bcm2708;

static int bcm2708_pl011_arm_init(SysBusDevice *dev, pl011_state *s)
{
    //~ sysbus_init_irq(dev, &s->irq);
    s->id = pl011_id_arm;
    s->chr = qemu_char_get_next_serial();

    s->read_trigger = 1;
    s->ifl = 0x12;
    s->cr = 0x300;
    s->flags = 0x90;
    if (s->chr) {
        qemu_chr_add_handlers(s->chr, pl011_can_receive, pl011_receive,
                              pl011_event, s);
    }
    vmstate_register(&dev->qdev, -1, &vmstate_pl011, s);
    return 0;
}

/* -------------------------------------------------------------------------- */

static int bcm2708_init(SysBusDevice *dev)
{
    BCM2708State *s = FROM_SYSBUS(BCM2708State, dev);
    MemoryRegion *sysmem = get_system_memory();
    DriveInfo *dinfo;
    int i;

    logout("\n");

    rpi->bcm2708 = s;

    //~ qdev_init_gpio_in(&dev->qdev, bcm2708_set_irq, 32);
    for (i = 0; i < 2; i++) {   // TODO
        sysbus_init_irq(dev, &s->ic.irq[i]);
    }
    //~ s->irq = 31;

    /* BCM2708. */
    memory_region_init_io(&s->iomem, &bcm2708_ops, s, "bcm2708",
                          IO_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    /* The BCM2835 includes an MMU which maps ARM physical addresses to
       bus addresses. The address space is splitted in 4 alias regions. */
    memory_region_init_alias(&s->alias_4, "bcm2708.4.alias",
                             &s->iomem, 0, IO_SIZE);
    memory_region_add_subregion(sysmem, 0x60000000, &s->alias_4);
    memory_region_init_alias(&s->alias_8, "bcm2708.8.alias",
                             &s->iomem, 0, IO_SIZE);
    memory_region_add_subregion(sysmem, 0xa0000000, &s->alias_8);
    memory_region_init_alias(&s->alias_c, "bcm2708.c.alias",
                             &s->iomem, 0, IO_SIZE);
    memory_region_add_subregion(sysmem, 0xe0000000, &s->alias_c);

    /* Interrupt controller. */
    memory_region_init_io(&s->ic.iomem, &bcm2708_armctrl_ops, s, "bcm2708.ic",
                          0x0400);
    memory_region_add_subregion(&s->iomem,
                                ARMCTRL_BASE - BCM2708_PERI_BASE,
                                &s->ic.iomem);

    /* System timer. */
    memory_region_init_io(&s->st.iomem, &bcm2708_st_ops, &s->st, "bcm2708.st",
                          0x1000);
    memory_region_add_subregion(&s->iomem,
                                ST_BASE - BCM2708_PERI_BASE,
                                &s->st.iomem);
    s->st.timer = qemu_new_timer_us(vm_clock, bcm2708_timer_tick, &s->st);
    s->st.clock = s->st.expire = qemu_get_clock_us(vm_clock);
    sysbus_init_irq(dev, &s->st.irq);
    bcm2708_timer_update(&s->st);

    /* UART0. */
    //~ sysbus_create_simple("bcm2708.pl011", UART0_BASE, s->parent[12]);
    memory_region_init_io(&s->uart0.iomem, &pl011_ops, &s->uart0, "bcm2708.uart0",
                          0x1000);
    memory_region_add_subregion(&s->iomem,
                                UART0_BASE - BCM2708_PERI_BASE,
                                &s->uart0.iomem);
    bcm2708_pl011_arm_init(dev, &s->uart0);

    /* Timer 0 and 1. */
    DeviceState *devState = qdev_create(NULL, "bcm2708.sp804");
    //~ qdev_prop_set_uint32(dev, "freq0", 150000000);
    //~ qdev_prop_set_uint32(dev, "freq1", 150000000);
    qdev_init_nofail(devState);
    SysBusDevice *busdev = sysbus_from_qdev(devState);
    busdev->mmio[0].addr = ARMCTRL_TIMER0_1_BASE;       // TODO: check
    memory_region_add_subregion(&s->iomem,
                                ARMCTRL_TIMER0_1_BASE - BCM2708_PERI_BASE,
                                busdev->mmio[0].memory);
    sysbus_init_irq(busdev, &s->ic.irq[0]);

    s->fb.ds = graphic_console_init(bcm2708_fb_update, bcm2708_fb_invalidate,
                                    bcm2708_fb_dump, bcm2708_fb_text_update,
                                    &s->fb);

    switch (16) {
    case 0:
        s->fb.dest_width = 0;
        break;

    case 8:
        s->fb.line_fn = s3c24xx_draw_fn_8;
        s->fb.dest_width = 1;
        break;

    case 15:
        s->fb.line_fn = s3c24xx_draw_fn_15;
        s->fb.dest_width = 2;
        break;

    case 16:
        s->fb.line_fn = s3c24xx_draw_fn_16;
        s->fb.dest_width = 2;
        break;

    case 24:
        s->fb.line_fn = s3c24xx_draw_fn_24;
        s->fb.dest_width = 3;
        break;

    case 32:
        s->fb.line_fn = s3c24xx_draw_fn_32;
        s->fb.dest_width = 4;
        break;

    default:
        fprintf(stderr, "%s: Bad color depth\n", __FUNCTION__);
        exit(1);
    }

    /* SD card. */
    dinfo = drive_get_next(IF_SD);
    s->emmc.card = sd_init(dinfo ? dinfo->bdrv : NULL, 0);
    s->emmc.enabled = dinfo ? bdrv_is_inserted(dinfo->bdrv) : 0;

#if 0
// TODO: use these functions:
int sd_do_command(SDState *sd, SDRequest *req,
                  uint8_t *response);
void sd_write_data(SDState *sd, uint8_t value);
uint8_t sd_read_data(SDState *sd);
void sd_set_cb(SDState *sd, qemu_irq readonly, qemu_irq insert);
int sd_data_ready(SDState *sd);
void sd_enable(SDState *sd, int enable);
#endif

    return 0;
}

static const VMStateDescription vmstate_bcm2708 = {
    .name = "bcm2708",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        //~ VMSTATE_UINT32(level, BCM2708State),
        //~ VMSTATE_UINT32(mask, BCM2708State),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2708_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    logout("\n");
    k->init = bcm2708_init;
    dc->no_user = 1;
    dc->vmsd = &vmstate_bcm2708;
    //~ dc->props = bcm2708_properties;
    //~ dc->reset = bcm2708_reset;
}

static TypeInfo bcm2708_info = {
    .name          = "bcm2708",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2708State),
    .class_init    = bcm2708_class_init,
};

static void raspi_register_types(void)
{
    logout("\n");
    type_register_static(&bcm2708_info);
}

type_init(raspi_register_types)



/* Board init. */

static void raspi_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    ARMCPU *cpu;
    MemoryRegion *sysmem = get_system_memory();

    logout("\n");

    rpi = g_new0(RaspberryPi, 1);

    if (!cpu_model) {
        cpu_model = "arm1176";
    }
    cpu = cpu_arm_init(cpu_model);
    if (!cpu) {
        logout("Unable to find CPU definition\n");
        exit(1);
    }

    /* Always allocate 256 MiB RAM. */
    memory_region_init_ram(&rpi->ram, "raspi.ram", RAM_SIZE);
    vmstate_register_ram_global(&rpi->ram);
    memory_region_add_subregion(sysmem, BCM2708_SDRAM_BASE, &rpi->ram);

    /* The BCM2835 includes an MMU which maps ARM physical addresses to
       bus addresses. The address space is splitted in 4 alias regions. */
    memory_region_init_alias(&rpi->ram_4_alias, "ram.4.alias",
                             &rpi->ram, 0, RAM_SIZE);
    memory_region_add_subregion(sysmem, 0x40000000, &rpi->ram_4_alias);
    memory_region_init_alias(&rpi->ram_8_alias, "ram.8.alias",
                             &rpi->ram, 0, RAM_SIZE);
    memory_region_add_subregion(sysmem, 0x80000000, &rpi->ram_8_alias);
    memory_region_init_alias(&rpi->ram_c_alias, "ram.c.alias",
                             &rpi->ram, 0, RAM_SIZE);
    memory_region_add_subregion(sysmem, 0xc0000000, &rpi->ram_c_alias);

    rpi->cpu_pic = arm_pic_init_cpu(cpu);

    //~ DeviceState *sysctl =
    sysbus_create_varargs("bcm2708", BCM2708_PERI_BASE,
                          rpi->cpu_pic[ARM_PIC_CPU_IRQ],
                          rpi->cpu_pic[ARM_PIC_CPU_FIQ],
                          NULL);

    //~ sysbus_connect_irq(sysbus_from_qdev(sysctl), 0, rpi->cpu_pic[ARM_PIC_CPU_IRQ]);

    if (ram_size > RAM_SIZE) {
        /* Limit the RAM size for Linux to the size of the physical RAM. */
        ram_size = RAM_SIZE;
    }

    rpi->binfo.ram_size = ram_size;
    rpi->binfo.kernel_filename = kernel_filename;
    rpi->binfo.kernel_cmdline = kernel_cmdline;
    rpi->binfo.initrd_filename = initrd_filename;
    rpi->binfo.board_id = 0x183;
    rpi->binfo.loader_start = BCM2708_SDRAM_BASE;
    rpi->binfo.nb_cpus = 1;
    arm_load_kernel(cpu, &rpi->binfo);
}

static QEMUMachine raspi_machine = {
    .name = "raspi",
    .desc = "ARM Raspberry PI (ARM1176)",
    .init = raspi_init,
    .use_scsi = 1,
    .no_parallel = true,
    //~ .use_virtcon:1,
    .no_floppy = true,
    .no_cdrom = true,
    //~ .default_machine_opts = "ram=256"
};

static void raspi_machine_init(void)
{
    logout("\n");
    qemu_register_machine(&raspi_machine);
}

machine_init(raspi_machine_init);

#if 0

pi@raspberrypi ~ $ cat /proc/interrupts
           CPU0
  3:     210938   ARMCTRL  BCM2708 Timer Tick
 52:          0   ARMCTRL  BCM2708 GPIO catchall handler
 65:          5   ARMCTRL  ARM Mailbox IRQ
 66:          1   ARMCTRL  VCHIQ doorbell
 75:   15792842   ARMCTRL  dwc_otg, dwc_otg_hcd:usb1
 77:       8921   ARMCTRL  bcm2708_sdhci (dma)
 83:         20   ARMCTRL  uart-pl011
 84:      11172   ARMCTRL  mmc0
Err:          0

pi@raspberrypi ~ $ cat /proc/iomem
00000000-0bffffff : System RAM
  00008000-003adfff : Kernel text
  003cc000-0043c397 : Kernel data
20000000-20000fff : bcm2708_vcio
20003000-20003fff : bcm2708_systemtimer
20007000-20007fff : bcm2708_dma.0
  20007000-20007fff : bcm2708_dma
20100000-201000ff : bcm2708_powerman.0
20200000-20200fff : bcm2708_gpio
20201000-20201fff : dev:f1
  20201000-20201fff : uart-pl011
20204000-202040ff : bcm2708_spi.0
20205000-202050ff : bcm2708_i2c.0
20300000-203000ff : bcm2708_sdhci.0
  20300000-203000ff : mmc0
20804000-208040ff : bcm2708_i2c.1
20980000-2099ffff : bcm2708_usb
  20980000-2099ffff : dwc_otg

TODO:

arch_hw_breakpoint_init - keine Debug-Architektur

mailbox0 from GPU to ARM
mailbox1 from ARM to GPU

Errata Manual:

* can used -> can be used
* it is easy sample -> it is easy to sample
* the before -> before
* kenel -> kernel

ARASAN SD3.0_Host_AHB_eMMC4.4_Usersguide_ver5.9_jan11_10.pdf
DWC_otg_databook.pdf
https://www.synopsys.com/dw/ipdir.php?ds=dwc_usb_2_0_hs_otg

#endif
