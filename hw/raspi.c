/*
 * ARM Raspberry Pi System emulation.
 *
 * Copyright (c) 2012 Stefan Weil
 *
 * Based on versatilepb.c.
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GNU GPL 2 or later.
 */

#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "net.h"
#include "sysemu.h"
//~ #include "i2c.h"
#include "boards.h"
#include "blockdev.h"
#include "exec-memory.h"
#include "flash.h"

/* Primary interrupt controller.  */

typedef struct
{
  SysBusDevice busdev;
  MemoryRegion iomem;
  uint32_t level;
  uint32_t mask;
  uint32_t pic_enable;
  qemu_irq parent[32];
  int irq;
} vpb_sic_state;

static const VMStateDescription vmstate_vpb_sic = {
    .name = "raspi_sic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(level, vpb_sic_state),
        VMSTATE_UINT32(mask, vpb_sic_state),
        VMSTATE_UINT32(pic_enable, vpb_sic_state),
        VMSTATE_END_OF_LIST()
    }
};

static void vpb_sic_update(vpb_sic_state *s)
{
    uint32_t flags;

    flags = s->level & s->mask;
    qemu_set_irq(s->parent[s->irq], flags != 0);
}

static void vpb_sic_update_pic(vpb_sic_state *s)
{
    int i;
    uint32_t mask;

    for (i = 21; i <= 30; i++) {
        mask = 1u << i;
        if (!(s->pic_enable & mask))
            continue;
        qemu_set_irq(s->parent[i], (s->level & mask) != 0);
    }
}

static void vpb_sic_set_irq(void *opaque, int irq, int level)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;
    if (level)
        s->level |= 1u << irq;
    else
        s->level &= ~(1u << irq);
    if (s->pic_enable & (1u << irq))
        qemu_set_irq(s->parent[irq], level);
    vpb_sic_update(s);
}

static uint64_t vpb_sic_read(void *opaque, target_phys_addr_t offset,
                             unsigned size)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;

    switch (offset >> 2) {
    case 0: /* STATUS */
        return s->level & s->mask;
    case 1: /* RAWSTAT */
        return s->level;
    case 2: /* ENABLE */
        return s->mask;
    case 4: /* SOFTINT */
        return s->level & 1;
    case 8: /* PICENABLE */
        return s->pic_enable;
    default:
        printf ("vpb_sic_read: Bad register offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void vpb_sic_write(void *opaque, target_phys_addr_t offset,
                          uint64_t value, unsigned size)
{
    vpb_sic_state *s = (vpb_sic_state *)opaque;

    switch (offset >> 2) {
    case 2: /* ENSET */
        s->mask |= value;
        break;
    case 3: /* ENCLR */
        s->mask &= ~value;
        break;
    case 4: /* SOFTINTSET */
        if (value)
            s->mask |= 1;
        break;
    case 5: /* SOFTINTCLR */
        if (value)
            s->mask &= ~1u;
        break;
    case 8: /* PICENSET */
        s->pic_enable |= (value & 0x7fe00000);
        vpb_sic_update_pic(s);
        break;
    case 9: /* PICENCLR */
        s->pic_enable &= ~value;
        vpb_sic_update_pic(s);
        break;
    default:
        printf ("vpb_sic_write: Bad register offset 0x%x\n", (int)offset);
        return;
    }
    vpb_sic_update(s);
}

static const MemoryRegionOps vpb_sic_ops = {
    .read = vpb_sic_read,
    .write = vpb_sic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int vpb_sic_init(SysBusDevice *dev)
{
    vpb_sic_state *s = FROM_SYSBUS(vpb_sic_state, dev);
    int i;

    qdev_init_gpio_in(&dev->qdev, vpb_sic_set_irq, 32);
    for (i = 0; i < 32; i++) {
        sysbus_init_irq(dev, &s->parent[i]);
    }
    s->irq = 31;
    memory_region_init_io(&s->iomem, &vpb_sic_ops, s, "vpb-sic", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

/* Board init.  */

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
    DeviceState *sysctl;

    if (!cpu_model) {
        cpu_model = "arm1176";
    }
    cpu = cpu_arm_init(cpu_model);
    if (!cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    memory_region_init_ram(ram, "raspi.ram", ram_size);
    vmstate_register_ram_global(ram);
    /* ??? RAM should repeat to fill physical memory space.  */
    memory_region_add_subregion(sysmem, 0x00000000, ram);

    memory_region_init_alias(ram_alias, "ram.alias", ram, 0, ram_size);
    memory_region_add_subregion(sysmem, 0xc0000000, ram_alias);

    sysctl = qdev_create(NULL, "realview_sysctl");
    qdev_prop_set_uint32(sysctl, "sys_id", 0x41007004);
    qdev_prop_set_uint32(sysctl, "proc_id", 0x02000000);
    qdev_init_nofail(sysctl);
    sysbus_mmio_map(sysbus_from_qdev(sysctl), 0, 0x10000000);

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
    .default_machine_opts = "ram=256"
};

static void raspi_machine_init(void)
{
    qemu_register_machine(&raspi_machine);
}

machine_init(raspi_machine_init);

static void vpb_sic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = vpb_sic_init;
    dc->no_user = 1;
    dc->vmsd = &vmstate_vpb_sic;
}

static TypeInfo vpb_sic_info = {
    .name          = "raspi_sic",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(vpb_sic_state),
    .class_init    = vpb_sic_class_init,
};

static void raspi_register_types(void)
{
    type_register_static(&vpb_sic_info);
}

type_init(raspi_register_types)
