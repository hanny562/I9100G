cmd_arch/arm/mm/pabort-v7.o := /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -Wp,-MD,arch/arm/mm/.pabort-v7.o.d  -nostdinc -isystem /opt/toolchains/arm-2009q3/bin/../lib/gcc/arm-none-linux-gnueabi/4.4.1/include -I/home/hanny/kernel/Lucifr/arch/arm/include -Iinclude  -include include/generated/autoconf.h -I/home/hanny/kernel/Lucifr/samsung/include -I/home/hanny/kernel/Lucifr/samsung/rfs_fsr/include -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2       -c -o arch/arm/mm/pabort-v7.o arch/arm/mm/pabort-v7.S

deps_arch/arm/mm/pabort-v7.o := \
  arch/arm/mm/pabort-v7.S \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/linkage.h \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/smp.h) \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/hwcap.h \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
    $(wildcard include/config/cpu/use/domains.h) \

arch/arm/mm/pabort-v7.o: $(deps_arch/arm/mm/pabort-v7.o)

$(deps_arch/arm/mm/pabort-v7.o):
