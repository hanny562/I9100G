cmd_firmware/mXT224.fw.gen.o := /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -Wp,-MD,firmware/.mXT224.fw.gen.o.d  -nostdinc -isystem /opt/toolchains/arm-2009q3/bin/../lib/gcc/arm-none-linux-gnueabi/4.4.1/include -I/home/hanny/kernel/Lucifr/arch/arm/include -Iinclude  -include include/generated/autoconf.h -I/home/hanny/kernel/Lucifr/samsung/include -I/home/hanny/kernel/Lucifr/samsung/rfs_fsr/include -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2       -c -o firmware/mXT224.fw.gen.o firmware/mXT224.fw.gen.S

deps_firmware/mXT224.fw.gen.o := \
  firmware/mXT224.fw.gen.S \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \

firmware/mXT224.fw.gen.o: $(deps_firmware/mXT224.fw.gen.o)

$(deps_firmware/mXT224.fw.gen.o):
