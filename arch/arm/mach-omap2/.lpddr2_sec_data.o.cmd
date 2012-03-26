cmd_arch/arm/mach-omap2/lpddr2_sec_data.o := /opt/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -Wp,-MD,arch/arm/mach-omap2/.lpddr2_sec_data.o.d  -nostdinc -isystem /opt/toolchains/arm-2009q3/bin/../lib/gcc/arm-none-linux-gnueabi/4.4.1/include -I/home/hanny/kernel/Lucifr/arch/arm/include -Iinclude  -include include/generated/autoconf.h -I/home/hanny/kernel/Lucifr/samsung/include -I/home/hanny/kernel/Lucifr/samsung/rfs_fsr/include -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -include arch/arm/plat-omap/include/plat/mux_t1_rev_r07.h -D_SAMSUNG_BOARD_NAME=t1 -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(lpddr2_sec_data)"  -D"KBUILD_MODNAME=KBUILD_STR(lpddr2_sec_data)"  -c -o arch/arm/mach-omap2/lpddr2_sec_data.o arch/arm/mach-omap2/lpddr2_sec_data.c

deps_arch/arm/mach-omap2/lpddr2_sec_data.o := \
  arch/arm/mach-omap2/lpddr2_sec_data.c \
  arch/arm/plat-omap/include/plat/mux_t1_rev_r07.h \
    $(wildcard include/config/video/mhl/v1.h) \
  arch/arm/mach-omap2/include/mach/emif.h \
    $(wildcard include/config/devct/1.h) \
    $(wildcard include/config/devwdt/32.h) \
  arch/arm/mach-omap2/include/mach/emif-44xx.h \
    $(wildcard include/config/2.h) \
  arch/arm/mach-omap2/include/mach/lpddr2-jedec.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/hanny/kernel/Lucifr/arch/arm/include/asm/posix_types.h \

arch/arm/mach-omap2/lpddr2_sec_data.o: $(deps_arch/arm/mach-omap2/lpddr2_sec_data.o)

$(deps_arch/arm/mach-omap2/lpddr2_sec_data.o):
