/*
 * ARM Raspberry Pi System emulation.
 *
 * Copyright (c) 2012 Stefan Weil
 *
 * This code is licensed under the GNU GPL 2 or later.
 *
 * This very basic emulation of Broadcom's BCM2835 media processor
 * works (partially as of 2012-07) with the Linux BCM2708 code.
 */

#include "qemu-common.h"
#include "hw/arm-misc.h"
#include "hw/devices.h"
#include "hw/boards.h"
#include "hw/flash.h"
#include "hw/sysbus.h"
#include "blockdev.h"
#include "exec-memory.h"
#include "net.h"
#include "sysemu.h"
//~ #include "i2c.h"

//~ $2 = {{virtual = 0xf200b000, pfn = 0x2000b, length = 0x1000, type = 0x0}, {virtual = 0xf2201000, pfn = 0x20201, length = 0x1000, type = 0x0}, {virtual = 0xf2215000,
    //~ pfn = 0x20215, length = 0x1000, type = 0x0}, {virtual = 0xf2007000, pfn = 0x20007, length = 0x1000, type = 0x0}, {virtual = 0xf2000000, pfn = 0x20000, length = 0x1000,
    //~ type = 0x0}, {virtual = 0xf2003000, pfn = 0x20003, length = 0x1000, type = 0x0}, {virtual = 0xf2980000, pfn = 0x20980, length = 0x20000, type = 0x0}, {virtual = 0xf2100000,
    //~ pfn = 0x20100, length = 0x1000, type = 0x0}, {virtual = 0xf2200000, pfn = 0x20200, length = 0x1000, type = 0x0}}

#define logout(fmt, ...) \
    fprintf(stderr, "RPI\t%-24s" fmt, __func__, ##__VA_ARGS__)

#define BCM2708
#include "hw/pl011.c"

/* Code copied from Linux arch/arm/mach-bcm2708/include/mach/platform.h. */

/* macros to get at IO space when running virtually */
#define IO_ADDRESS(x) (((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define BCM2708_SDRAM_BASE 0x00000000

// TODO: 0x7e000000?
#define BCM2708_PERI_BASE  0x20000000
#define ST_BASE            (BCM2708_PERI_BASE + 0x3000)   /* System Timer */
#define DMA_BASE           (BCM2708_PERI_BASE + 0x7000)   /* DMA controller */
#define ARM_BASE           (BCM2708_PERI_BASE + 0xB000)   /* BCM2708 ARM control block */
#define PM_BASE            (BCM2708_PERI_BASE + 0x100000) /* Power Management, Reset controller and Watchdog registers */
#define GPIO_BASE          (BCM2708_PERI_BASE + 0x200000) /* GPIO */
#define UART0_BASE         (BCM2708_PERI_BASE + 0x201000) /* Uart 0 */
#define MMCI0_BASE         (BCM2708_PERI_BASE + 0x202000) /* MMC interface */
#define UART1_BASE         (BCM2708_PERI_BASE + 0x215000) /* Uart 1 */
#define EMMC_BASE          (BCM2708_PERI_BASE + 0x300000) /* eMMC interface */
#define SMI_BASE           (BCM2708_PERI_BASE + 0x600000) /* SMI */
#define USB_BASE           (BCM2708_PERI_BASE + 0x980000) /* DTC_OTG USB controller */
#define MCORE_BASE         (BCM2708_PERI_BASE + 0x0000)   /* Fake frame buffer device (actually the multicore sync block*/

#define ARMCTRL_BASE             (ARM_BASE + 0x000)
#define ARMCTRL_IC_BASE          (ARM_BASE + 0x200)           /* ARM interrupt controller */
#define ARMCTRL_TIMER0_1_BASE    (ARM_BASE + 0x400)           /* Timer 0 and 1 */
#define ARMCTRL_0_SBM_BASE       (ARM_BASE + 0x800)           /* User 0 (ARM)'s Semaphores Doorbells and Mailboxes */

#define ARM_IRQ1_BASE                  0
#define INTERRUPT_TIMER0               (ARM_IRQ1_BASE + 0)
#define INTERRUPT_TIMER1               (ARM_IRQ1_BASE + 1)
#define INTERRUPT_TIMER2               (ARM_IRQ1_BASE + 2)
#define INTERRUPT_TIMER3               (ARM_IRQ1_BASE + 3)
#define INTERRUPT_CODEC0               (ARM_IRQ1_BASE + 4)
#define INTERRUPT_CODEC1               (ARM_IRQ1_BASE + 5)
#define INTERRUPT_CODEC2               (ARM_IRQ1_BASE + 6)
#define INTERRUPT_VC_JPEG              (ARM_IRQ1_BASE + 7)
#define INTERRUPT_ISP                  (ARM_IRQ1_BASE + 8)
#define INTERRUPT_VC_USB               (ARM_IRQ1_BASE + 9)
#define INTERRUPT_VC_3D                (ARM_IRQ1_BASE + 10)
#define INTERRUPT_TRANSPOSER           (ARM_IRQ1_BASE + 11)
#define INTERRUPT_MULTICORESYNC0       (ARM_IRQ1_BASE + 12)
#define INTERRUPT_MULTICORESYNC1       (ARM_IRQ1_BASE + 13)
#define INTERRUPT_MULTICORESYNC2       (ARM_IRQ1_BASE + 14)
#define INTERRUPT_MULTICORESYNC3       (ARM_IRQ1_BASE + 15)
#define INTERRUPT_DMA0                 (ARM_IRQ1_BASE + 16)
#define INTERRUPT_DMA1                 (ARM_IRQ1_BASE + 17)
#define INTERRUPT_VC_DMA2              (ARM_IRQ1_BASE + 18)
#define INTERRUPT_VC_DMA3              (ARM_IRQ1_BASE + 19)
#define INTERRUPT_DMA4                 (ARM_IRQ1_BASE + 20)
#define INTERRUPT_DMA5                 (ARM_IRQ1_BASE + 21)
#define INTERRUPT_DMA6                 (ARM_IRQ1_BASE + 22)
#define INTERRUPT_DMA7                 (ARM_IRQ1_BASE + 23)
#define INTERRUPT_DMA8                 (ARM_IRQ1_BASE + 24)
#define INTERRUPT_DMA9                 (ARM_IRQ1_BASE + 25)
#define INTERRUPT_DMA10                (ARM_IRQ1_BASE + 26)
#define INTERRUPT_DMA11                (ARM_IRQ1_BASE + 27)
#define INTERRUPT_DMA12                (ARM_IRQ1_BASE + 28)
#define INTERRUPT_AUX                  (ARM_IRQ1_BASE + 29)
#define INTERRUPT_ARM                  (ARM_IRQ1_BASE + 30)
#define INTERRUPT_VPUDMA               (ARM_IRQ1_BASE + 31)

#define ARM_IRQ2_BASE                  32
#define INTERRUPT_HOSTPORT             (ARM_IRQ2_BASE + 0)
#define INTERRUPT_VIDEOSCALER          (ARM_IRQ2_BASE + 1)
#define INTERRUPT_CCP2TX               (ARM_IRQ2_BASE + 2)
#define INTERRUPT_SDC                  (ARM_IRQ2_BASE + 3)
#define INTERRUPT_DSI0                 (ARM_IRQ2_BASE + 4)
#define INTERRUPT_AVE                  (ARM_IRQ2_BASE + 5)
#define INTERRUPT_CAM0                 (ARM_IRQ2_BASE + 6)
#define INTERRUPT_CAM1                 (ARM_IRQ2_BASE + 7)
#define INTERRUPT_HDMI0                (ARM_IRQ2_BASE + 8)
#define INTERRUPT_HDMI1                (ARM_IRQ2_BASE + 9)
#define INTERRUPT_PIXELVALVE1          (ARM_IRQ2_BASE + 10)
#define INTERRUPT_I2CSPISLV            (ARM_IRQ2_BASE + 11)
#define INTERRUPT_DSI1                 (ARM_IRQ2_BASE + 12)
#define INTERRUPT_PWA0                 (ARM_IRQ2_BASE + 13)
#define INTERRUPT_PWA1                 (ARM_IRQ2_BASE + 14)
#define INTERRUPT_CPR                  (ARM_IRQ2_BASE + 15)
#define INTERRUPT_SMI                  (ARM_IRQ2_BASE + 16)
#define INTERRUPT_GPIO0                (ARM_IRQ2_BASE + 17)
#define INTERRUPT_GPIO1                (ARM_IRQ2_BASE + 18)
#define INTERRUPT_GPIO2                (ARM_IRQ2_BASE + 19)
#define INTERRUPT_GPIO3                (ARM_IRQ2_BASE + 20)
#define INTERRUPT_VC_I2C               (ARM_IRQ2_BASE + 21)
#define INTERRUPT_VC_SPI               (ARM_IRQ2_BASE + 22)
#define INTERRUPT_VC_I2SPCM            (ARM_IRQ2_BASE + 23)
#define INTERRUPT_VC_SDIO              (ARM_IRQ2_BASE + 24)
#define INTERRUPT_VC_UART              (ARM_IRQ2_BASE + 25)
#define INTERRUPT_SLIMBUS              (ARM_IRQ2_BASE + 26)
#define INTERRUPT_VEC                  (ARM_IRQ2_BASE + 27)
#define INTERRUPT_CPG                  (ARM_IRQ2_BASE + 28)
#define INTERRUPT_RNG                  (ARM_IRQ2_BASE + 29)
#define INTERRUPT_VC_ARASANSDIO        (ARM_IRQ2_BASE + 30)
#define INTERRUPT_AVSPMON              (ARM_IRQ2_BASE + 31)

#define ARM_IRQ0_BASE                  64
#define INTERRUPT_ARM_TIMER            (ARM_IRQ0_BASE + 0)
#define INTERRUPT_ARM_MAILBOX          (ARM_IRQ0_BASE + 1)
#define INTERRUPT_ARM_DOORBELL_0       (ARM_IRQ0_BASE + 2)
#define INTERRUPT_ARM_DOORBELL_1       (ARM_IRQ0_BASE + 3)
#define INTERRUPT_VPU0_HALTED          (ARM_IRQ0_BASE + 4)
#define INTERRUPT_VPU1_HALTED          (ARM_IRQ0_BASE + 5)
#define INTERRUPT_ILLEGAL_TYPE0        (ARM_IRQ0_BASE + 6)
#define INTERRUPT_ILLEGAL_TYPE1        (ARM_IRQ0_BASE + 7)
#define INTERRUPT_PENDING1             (ARM_IRQ0_BASE + 8)
#define INTERRUPT_PENDING2             (ARM_IRQ0_BASE + 9)
#define INTERRUPT_JPEG                 (ARM_IRQ0_BASE + 10)
#define INTERRUPT_USB                  (ARM_IRQ0_BASE + 11)
#define INTERRUPT_3D                   (ARM_IRQ0_BASE + 12)
#define INTERRUPT_DMA2                 (ARM_IRQ0_BASE + 13)
#define INTERRUPT_DMA3                 (ARM_IRQ0_BASE + 14)
#define INTERRUPT_I2C                  (ARM_IRQ0_BASE + 15)
#define INTERRUPT_SPI                  (ARM_IRQ0_BASE + 16)
#define INTERRUPT_I2SPCM               (ARM_IRQ0_BASE + 17)
#define INTERRUPT_SDIO                 (ARM_IRQ0_BASE + 18)
#define INTERRUPT_UART                 (ARM_IRQ0_BASE + 19)
#define INTERRUPT_ARASANSDIO           (ARM_IRQ0_BASE + 20)

// TODO: 21?
#define MAXIRQNUM                      (32 + 32 + 20)
#define MAXFIQNUM                      (32 + 32 + 20)

#define MAX_TIMER                       2
#define MAX_PERIOD                      699050
#define TICKS_PER_uSEC                  1

/*
 *  These are useconds NOT ticks.
 *
 */
#define mSEC_1                          1000
#define mSEC_5                          (mSEC_1 * 5)
#define mSEC_10                         (mSEC_1 * 10)
#define mSEC_25                         (mSEC_1 * 25)
#define SEC_1                           (mSEC_1 * 1000)

/*
 * Watchdog
 */
#define PM_RSTC                        (PM_BASE+0x1c)
#define PM_WDOG                        (PM_BASE+0x24)

#define PM_WDOG_RESET                  0000000000
#define PM_PASSWORD                    0x5a000000
#define PM_WDOG_TIME_SET               0x000fffff
#define PM_RSTC_WRCFG_CLR              0xffffffcf
#define PM_RSTC_WRCFG_SET              0x00000030
#define PM_RSTC_WRCFG_FULL_RESET       0x00000020
#define PM_RSTC_RESET                  0x00000102

/* Linux code ends here. */

#define SZ_4K   4096

/* Board init. */

static struct arm_boot_info raspi_binfo;

static void raspi_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    ARMCPU *cpu;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *ram_alias = g_new(MemoryRegion, 1);
    //~ DeviceState *sysctl;

    logout("\n");

    if (!cpu_model) {
        cpu_model = "arm1176";
    }
    cpu = cpu_arm_init(cpu_model);
    if (!cpu) {
        logout("Unable to find CPU definition\n");
        exit(1);
    }

    /* Ignore the RAM size argument and use always the standard size. */
    ram_size = 256 * MiB;

    memory_region_init_ram(ram, "raspi.ram", ram_size);
    vmstate_register_ram_global(ram);
    /* ??? RAM should repeat to fill physical memory space.  */
    memory_region_add_subregion(sysmem, BCM2708_SDRAM_BASE, ram);

    memory_region_init_alias(ram_alias, "ram.alias", ram, 0, ram_size);
    memory_region_add_subregion(sysmem, 0xc0000000, ram_alias);

    //~ dev =
    sysbus_create_simple("bcm2708", BCM2708_PERI_BASE, NULL);

    //~ sysctl = qdev_create(NULL, "realview_sysctl");
    //~ qdev_prop_set_uint32(sysctl, "sys_id", 0x41007004);
    //~ qdev_prop_set_uint32(sysctl, "proc_id", 0x02000000);
    //~ qdev_init_nofail(sysctl);
    //~ sysbus_mmio_map(sysbus_from_qdev(sysctl), 0, 0x10000000);

    raspi_binfo.ram_size = ram_size;
    raspi_binfo.kernel_filename = kernel_filename;
    raspi_binfo.kernel_cmdline = kernel_cmdline;
    raspi_binfo.initrd_filename = initrd_filename;
    raspi_binfo.board_id = 0x183;
    arm_load_kernel(cpu, &raspi_binfo);
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
    //~ no_sdcard:1;
    //~ .default_machine_opts = "ram=256"
};

static void raspi_machine_init(void)
{
    logout("\n");
    qemu_register_machine(&raspi_machine);
}

machine_init(raspi_machine_init);

typedef struct {
    QEMUTimer *timer;
    int64_t expire;
    uint64_t clock;
    uint32_t dt;
    uint32_t cs;
    uint32_t c[4];
} BCM2708SystemTimer;

typedef struct
{
    SysBusDevice busdev;
    MemoryRegion iomem;
    MemoryRegion ic_mem;
    MemoryRegion st_mem;
    BCM2708SystemTimer st;      /* system timer */
    pl011_state uart0;
    uint32_t level;
    uint32_t mask;
    uint32_t pic_enable;
    qemu_irq parent[32];
    int irq;
} BCM2708State;

static const VMStateDescription vmstate_bcm2708 = {
    .name = "bcm2708",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        //~ VMSTATE_UINT32(level, BCM2708State),
        //~ VMSTATE_UINT32(mask, BCM2708State),
        //~ VMSTATE_UINT32(pic_enable, BCM2708State),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t bcm2708_timer_clock(BCM2708State *s)
{
    uint64_t cl = s->st.clock;
    int64_t dt = s->st.expire - qemu_get_clock_us(vm_clock);
    if (dt < 0) {
        dt = 0;
    }
    cl += s->st.dt - dt;
    return cl;
}

static void bcm2708_timer_next(BCM2708State *s)
{
    uint8_t i;
    uint32_t clo = (uint32_t)s->st.clock + 1;
    uint32_t dt_min = UINT32_MAX;
    for (i = 0; i < 4; i++) {
        uint32_t dt = s->st.c[i] - clo;
        if (dt != 0 && dt < dt_min) {
            dt_min = dt;
        }
    }
    s->st.dt = dt_min;
    s->st.expire += dt_min;
    logout("wait until %" PRId64 " us\n", s->st.expire);
    qemu_mod_timer(s->st.timer, s->st.expire);
}

static void bcm2708_timer_tick(void *opaque)
{
    BCM2708State *s = opaque;
    uint32_t clo;
    uint8_t cs = s->st.cs;
    uint8_t i;
    logout("%u us expired\n", s->st.dt);
    s->st.clock += s->st.dt;
    clo = (uint32_t)s->st.clock;
    for (i = 0; i < 4; i++) {
        if ((cs ^ (1 << i)) && (s->st.c[i] == clo)) {
            s->st.cs |= (1 << i);
            //~ qemu_irq_lower(s->davint);
        }
    }
    bcm2708_timer_next(s);
}

/* DMA. */

static uint32_t bcm2708_dma_read(BCM2708State *s, unsigned offset)
{
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
        return value;
    }
    logout("offset=0x%02x, value=0x%04x\n", offset, value);
    return value;
}

static void bcm2708_dma_write(BCM2708State *s, unsigned offset, uint32_t value)
{
    //~ logout("offset=0x%02x, value=0x%04x\n", offset, value);
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
    }
}

/* ARM Interrupt controller. */

static uint64_t bcm2708_ic_read(void *opaque, target_phys_addr_t offset,
                                unsigned size)
{
    //~ BCM2708State *s = opaque;
    uint32_t value = 0;
    assert(size == 4);
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x0000 (TODO)\n", offset);
    }
    return value;
}

static void bcm2708_ic_write(void *opaque, unsigned offset, uint64_t value,
                             unsigned size)
{
    //~ BCM2708State *s = opaque;
    //~ bcm2708_write: Bad register offset 0xb210
    //~ bcm2708_write           offset=b218
    assert(size == 4);
    switch (offset) {
    case 0x00: /* IRQ basic pending */
    case 0x04: /* IRQ pending 1 */
    case 0x08: /* IRQ pending 2 */
    case 0x0c: /* FIQ control */
    case 0x10: /* Enable IRQs 1 */
    case 0x14: /* Enable IRQs 2 */
    case 0x18: /* Enable Basic IRQs */
    case 0x1c: /* Disable IRQs 1 */
    case 0x20: /* Disable IRQs 2 */
    case 0x24: /* Disable Basic IRQs */
    default:
        logout("offset=0x%02x, value=0x%04" PRIx64 " (TODO)\n", offset, value);
    }
}

/* System timer. */

static uint64_t bcm2708_st_read(void *opaque, target_phys_addr_t offset,
                                unsigned size)
{
    BCM2708State *s = opaque;
    uint32_t value = 0;
    assert(size == 4);
    switch (offset) {
    case 0x04: /* CLO */
        value = (uint32_t)bcm2708_timer_clock(s);
        break;
    case 0x08: /* CHI */
        value = (uint32_t)(bcm2708_timer_clock(s) >> 32);
        break;
    case 0x0c ... 0x18: /* C0, C1, C2, C3 */
        value = s->st.c[(offset - 0x0c) / 4];
        break;
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
        return value;
    }
    logout("offset=0x%02x, value=0x%04x\n", offset, value);
    return value;
}

static void bcm2708_st_write(void *opaque, target_phys_addr_t offset,
                             uint64_t value, unsigned size)
{
    BCM2708State *s = opaque;
    assert(size == 4);
    switch (offset) {
    case 0x04: /* CLO */
    case 0x08: /* CHI */
        logout("offset=0x%02x, value=0x%04" PRIx64 " (ignored)\n",
               offset, value);
        break;
    case 0x0c ... 0x18: /* C0, C1, C2, C3 */
        s->st.c[(offset - 0x0c) / 4] = value;
        logout("offset=0x%02x, value=0x%04" PRIx64 " (C%u, TODO)\n",
               offset, value, (offset - 0x0c) / 4);
        break;
    default:
        logout("offset=0x%02x, value=0x%04" PRIx64 " (TODO)\n", offset, value);
    }
}

/* ARM SP804 timer with some modifications. */

#if 0
static void bcm2708_timer0_1_write(BCM2708State *s, unsigned offset,
                                   uint32_t value)
{
    //~ bcm2708_write: Bad register offset 0xb408
    //~ logout("offset=0x%02x, value=0x%04x\n", offset, value);
    switch (offset) {
    case 0x08: /* Control */
        logout("offset=0x%02x, value=0x%04x (Control, TODO)\n", offset, value);
        break;
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
    }
}
#endif

/* UART 0. */

static uint32_t bcm2708_uart0_read(BCM2708State *s, unsigned offset)
{
    //~ RPI     bcm2708_read            offset=201fe0
    //~ RPI     bcm2708_read            offset=201fe4
    //~ RPI     bcm2708_read            offset=201fe8
    //~ RPI     bcm2708_read            offset=201fec
    //~ RPI     bcm2708_read            offset=201ff0
    //~ RPI     bcm2708_read            offset=201ff4
    //~ RPI     bcm2708_read            offset=201ff8
    //~ RPI     bcm2708_read            offset=201ffc
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
        return value;
    }
    logout("offset=0x%02x, value=0x%04x\n", offset, value);
    return value;
}

static uint32_t bcm2708_0_sbm_read(BCM2708State *s, unsigned offset)
{
    //~ RPI     bcm2708_write           offset=b89c
    //~ RPI     bcm2708_read            offset=b898
    //~ RPI     bcm2708_write           offset=b8a0
    //~ RPI     bcm2708_read            offset=b898
    //~ RPI     bcm2708_write           offset=b8a0
    uint32_t value = 0;
    switch (offset) {
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, value);
        return value;
    }
    logout("offset=0x%02x, value=0x%04x\n", offset, value);
    return value;
}

#define IO(offset) (offset + BCM2708_PERI_BASE)

static uint64_t bcm2708_read(void *opaque, target_phys_addr_t offset,
                             unsigned size)
{
    BCM2708State *s = opaque;
    uint32_t value = 0;

    assert(size == 4);

    switch (IO(offset)) {
#if 0
    case ST_BASE ... ST_BASE + 0x18:
        value = bcm2708_st_read(s, IO(offset) - ST_BASE);
        break;
#endif
    case DMA_BASE ... DMA_BASE + SZ_4K - 1:
        value = bcm2708_dma_read(s, IO(offset) - DMA_BASE);
        break;
    case UART0_BASE ... UART0_BASE + 0xffc:
        value = bcm2708_uart0_read(s, IO(offset) - UART0_BASE);
        break;
    case ARMCTRL_0_SBM_BASE ... ARMCTRL_0_SBM_BASE + 0xa0:
        value = bcm2708_0_sbm_read(s, IO(offset) - ARMCTRL_0_SBM_BASE);
        break;
    default:
        logout("offset=0x%02x, value=0x0000 (TODO)\n", offset);
    }
    return value;
}

static void bcm2708_write(void *opaque, target_phys_addr_t offset,
                          uint64_t value, unsigned size)
{
    BCM2708State *s = opaque;

    assert(size == 4);

    switch (IO(offset)) {
#if 0
    case ST_BASE ... ST_BASE + 0x18:
        bcm2708_st_write(s, IO(offset) - ST_BASE, value);
        break;
#endif
    case DMA_BASE ... DMA_BASE + SZ_4K - 1:
        bcm2708_dma_write(s, IO(offset) - DMA_BASE, value);
        break;
#if 0
    case ARMCTRL_IC_BASE ... ARMCTRL_IC_BASE + 0x24:
        bcm2708_ic_write(s, IO(offset) - ARMCTRL_IC_BASE, value);
        break;
    case ARMCTRL_TIMER0_1_BASE ... ARMCTRL_TIMER0_1_BASE + 0x08:
        bcm2708_timer0_1_write(s, IO(offset) - ARMCTRL_TIMER0_1_BASE, value);
        break;
#endif
    default:
        logout("offset=0x%02x, value=0x%04x (TODO)\n", offset, (unsigned)value);
    }
    //~ bcm2708_update(s);
}

static const MemoryRegionOps bcm2708_ic_ops = {
    .read = bcm2708_ic_read,
    .write = bcm2708_ic_write,
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

/* -------------------------------------------------------------------------- */

static BCM2708State *bcm2708;

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
    int i;

    logout("\n");

    bcm2708 = s;

    //~ qdev_init_gpio_in(&dev->qdev, bcm2708_set_irq, 32);
    for (i = 0; i < 32; i++) {
        sysbus_init_irq(dev, &s->parent[i]);
    }
    s->irq = 31;

    memory_region_init_io(&s->iomem, &bcm2708_ops, s, "bcm2708",
                          //~ ARMCTRL_BASE - BCM2708_PERI_BASE);
                          0x1000 + USB_BASE - BCM2708_PERI_BASE);
    sysbus_init_mmio(dev, &s->iomem);

    memory_region_init_io(&s->ic_mem, &bcm2708_ic_ops, s, "bcm2708.ic",
                          0x1000);
    memory_region_add_subregion(&s->iomem,
                                ARMCTRL_IC_BASE - BCM2708_PERI_BASE,
                                &s->ic_mem);
    //~ sysbus_init_mmio(dev, &s->ic_mem);

    memory_region_init_io(&s->st_mem, &bcm2708_st_ops, s, "bcm2708.st",
                          0x1000);
    memory_region_add_subregion(&s->iomem,
                                ST_BASE - BCM2708_PERI_BASE,
                                &s->st_mem);

    memory_region_init_io(&s->uart0.iomem, &pl011_ops, s, "bcm2708.uart0",
                          0x1000);
    memory_region_add_subregion(&s->iomem,
                                UART0_BASE - BCM2708_PERI_BASE,
                                &s->uart0.iomem);
    bcm2708_pl011_arm_init(dev, &s->uart0);

    s->st.timer = qemu_new_timer_us(vm_clock, bcm2708_timer_tick, s);
    s->st.expire = qemu_get_clock_us(vm_clock);
    bcm2708_timer_next(s);

    //~ sysbus_create_simple("pl011", 0x07e20100, s->parent[12]);
    //~ sysbus_create_simple("bcm2708.pl011", UART0_BASE, s->parent[12]);
    //~ sysbus_create_simple("sp804", 0x07e00400, s->parent[12]);
    sysbus_create_simple("sp804", ARMCTRL_TIMER0_1_BASE, s->parent[12]);

    return 0;
}

static void bcm2708_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    logout("\n");
    k->init = bcm2708_init;
    dc->no_user = 1;
    dc->vmsd = &vmstate_bcm2708;
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
