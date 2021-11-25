# Common mips*-softmmu CONFIG defines

# CONFIG_SEMIHOSTING is always required on this architecture
CONFIG_SEMIHOSTING=y

CONFIG_ISA_BUS=y
CONFIG_PCI=y
CONFIG_PCI_DEVICES=y

# Systems.
# TODO: Fix AR7
#CONFIG_AR7=y

CONFIG_VGA_ISA=y
CONFIG_VGA_ISA_MM=y
CONFIG_VGA_CIRRUS=y
CONFIG_VMWARE_VGA=y
CONFIG_SERIAL=y
CONFIG_SERIAL_ISA=y
CONFIG_PARALLEL=y
CONFIG_I8254=y
CONFIG_PCSPK=y
CONFIG_PCKBD=y
CONFIG_FDC=y
CONFIG_ACPI=y
CONFIG_ACPI_PIIX4=y
CONFIG_APM=y
CONFIG_I8257=y
CONFIG_PIIX4=y
CONFIG_IDE_ISA=y
CONFIG_IDE_PIIX=y
CONFIG_PFLASH_CFI01=y
CONFIG_PFLASH_CFI02=y
CONFIG_G364FB=y
CONFIG_I8259=y
CONFIG_MC146818RTC=y
CONFIG_EMPTY_SLOT=y
CONFIG_MIPS_CPS=y
CONFIG_MIPS_ITU=y
CONFIG_MALTA=y
CONFIG_PCNET_PCI=y
CONFIG_MIPSSIM=y
CONFIG_ACPI_SMBUS=y
CONFIG_SMBUS_EEPROM=y
CONFIG_TEST_DEVICES=y
