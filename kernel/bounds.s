	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 2
	.eabi_attribute 30, 4
	.eabi_attribute 18, 4
	.file	"bounds.c"
@ GNU C (Sourcery G++ Lite 2009q3-67) version 4.4.1 (arm-none-linux-gnueabi)
@	compiled by GNU C version 4.3.2, GMP version 4.3.1, MPFR version 2.4.1-p5.
@ GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
@ options passed:  -nostdinc -I/home/hanny/kernel/Lucifr/arch/arm/include
@ -Iinclude -I/home/hanny/kernel/Lucifr/samsung/include
@ -I/home/hanny/kernel/Lucifr/samsung/rfs_fsr/include
@ -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -iprefix
@ /opt/toolchains/arm-2009q3/bin/../lib/gcc/arm-none-linux-gnueabi/4.4.1/
@ -isysroot /opt/toolchains/arm-2009q3/bin/../arm-none-linux-gnueabi/libc
@ -D__KERNEL__ -D__LINUX_ARM_ARCH__=7 -Uarm -D_SAMSUNG_BOARD_NAME=t1
@ -DKBUILD_STR(s)=#s -DKBUILD_BASENAME=KBUILD_STR(bounds)
@ -DKBUILD_MODNAME=KBUILD_STR(bounds) -isystem
@ /opt/toolchains/arm-2009q3/bin/../lib/gcc/arm-none-linux-gnueabi/4.4.1/include
@ -include include/generated/autoconf.h -include
@ arch/arm/plat-omap/include/plat/mux_t1_rev_r07.h -MD kernel/.bounds.s.d
@ kernel/bounds.c -mlittle-endian -marm -mapcs -mno-sched-prolog
@ -mabi=aapcs-linux -mno-thumb-interwork -march=armv7-a -msoft-float
@ -auxbase-strip kernel/bounds.s -g -Os -Wall -Wundef -Wstrict-prototypes
@ -Wno-trigraphs -Werror-implicit-function-declaration -Wno-format-security
@ -Wframe-larger-than=1024 -Wdeclaration-after-statement -Wno-pointer-sign
@ -fno-strict-aliasing -fno-common -fno-delete-null-pointer-checks
@ -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls
@ -fno-strict-overflow -fconserve-stack -fverbose-asm
@ options enabled:  -falign-loops -fargument-alias -fauto-inc-dec
@ -fbranch-count-reg -fcaller-saves -fcprop-registers -fcrossjumping
@ -fcse-follow-jumps -fdefer-pop -fearly-inlining
@ -feliminate-unused-debug-types -fexpensive-optimizations
@ -fforward-propagate -ffunction-cse -fgcse -fgcse-lm
@ -fguess-branch-probability -fident -fif-conversion -fif-conversion2
@ -findirect-inlining -finline -finline-functions
@ -finline-functions-called-once -finline-small-functions -fipa-cp
@ -fipa-pure-const -fipa-reference -fira-share-save-slots
@ -fira-share-spill-slots -fivopts -fkeep-static-consts
@ -fleading-underscore -fmath-errno -fmerge-constants -fmerge-debug-strings
@ -foptimize-register-move -fpeephole -fpeephole2 -freg-struct-return
@ -fregmove -freorder-blocks -freorder-functions -frerun-cse-after-loop
@ -fsched-interblock -fsched-spec -fsched-stalled-insns-dep
@ -fschedule-insns -fschedule-insns2 -fsection-anchors -fsigned-zeros
@ -fsplit-ivs-in-unroller -fsplit-wide-types -fthread-jumps
@ -ftoplevel-reorder -ftrapping-math -ftree-builtin-call-dce -ftree-ccp
@ -ftree-ch -ftree-copy-prop -ftree-copyrename -ftree-cselim -ftree-dce
@ -ftree-dominator-opts -ftree-dse -ftree-fre -ftree-loop-im
@ -ftree-loop-ivcanon -ftree-loop-optimize -ftree-parallelize-loops=
@ -ftree-reassoc -ftree-scev-cprop -ftree-sink -ftree-sra
@ -ftree-switch-conversion -ftree-ter -ftree-vect-loop-version -ftree-vrp
@ -funit-at-a-time -fvar-tracking -fverbose-asm -fzero-initialized-in-bss
@ -mapcs-frame -mglibc -mlittle-endian

	.section	.debug_abbrev,"",%progbits
.Ldebug_abbrev0:
	.section	.debug_info,"",%progbits
.Ldebug_info0:
	.section	.debug_line,"",%progbits
.Ldebug_line0:
	.text
.Ltext0:
@ Compiler executable checksum: 74011fea7f66b1cda6b19363b6694363

	.align	2
	.global	foo
	.type	foo, %function
foo:
.LFB0:
	.file 1 "kernel/bounds.c"
	.loc 1 14 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp	@,
.LCFI0:
	stmfd	sp!, {fp, ip, lr, pc}	@,
.LCFI1:
	sub	fp, ip, #4	@,,
.LCFI2:
	.loc 1 16 0
#APP
@ 16 "kernel/bounds.c" 1
	
->NR_PAGEFLAGS #23 __NR_PAGEFLAGS	@
@ 0 "" 2
	.loc 1 17 0
@ 17 "kernel/bounds.c" 1
	
->MAX_NR_ZONES #3 __MAX_NR_ZONES	@
@ 0 "" 2
	.loc 1 19 0
	ldmfd	sp, {fp, sp, pc}	@
.LFE0:
	.size	foo, .-foo
	.section	.debug_frame,"",%progbits
.Lframe0:
	.4byte	.LECIE0-.LSCIE0
.LSCIE0:
	.4byte	0xffffffff
	.byte	0x1
	.ascii	"\000"
	.uleb128 0x1
	.sleb128 -4
	.byte	0xe
	.byte	0xc
	.uleb128 0xd
	.uleb128 0x0
	.align	2
.LECIE0:
.LSFDE0:
	.4byte	.LEFDE0-.LASFDE0
.LASFDE0:
	.4byte	.Lframe0
	.4byte	.LFB0
	.4byte	.LFE0-.LFB0
	.byte	0x4
	.4byte	.LCFI0-.LFB0
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI1-.LCFI0
	.byte	0x11
	.uleb128 0xe
	.sleb128 2
	.byte	0x11
	.uleb128 0xd
	.sleb128 3
	.byte	0x11
	.uleb128 0xb
	.sleb128 4
	.byte	0x4
	.4byte	.LCFI2-.LCFI1
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE0:
	.text
.Letext0:
	.section	.debug_loc,"",%progbits
.Ldebug_loc0:
.LLST0:
	.4byte	.LFB0-.Ltext0
	.4byte	.LCFI0-.Ltext0
	.2byte	0x1
	.byte	0x5d
	.4byte	.LCFI0-.Ltext0
	.4byte	.LCFI2-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	.LCFI2-.Ltext0
	.4byte	.LFE0-.Ltext0
	.2byte	0x2
	.byte	0x7b
	.sleb128 4
	.4byte	0x0
	.4byte	0x0
	.file 2 "include/linux/page-flags.h"
	.file 3 "include/linux/mmzone.h"
	.section	.debug_info
	.4byte	0x17a
	.2byte	0x2
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.4byte	.LASF48
	.byte	0x1
	.4byte	.LASF49
	.4byte	.LASF50
	.4byte	.Ltext0
	.4byte	.Letext0
	.4byte	.Ldebug_line0
	.uleb128 0x2
	.byte	0x1
	.byte	0x6
	.4byte	.LASF0
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF1
	.uleb128 0x2
	.byte	0x2
	.byte	0x5
	.4byte	.LASF2
	.uleb128 0x2
	.byte	0x2
	.byte	0x7
	.4byte	.LASF3
	.uleb128 0x3
	.byte	0x4
	.byte	0x5
	.ascii	"int\000"
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF4
	.uleb128 0x2
	.byte	0x8
	.byte	0x5
	.4byte	.LASF5
	.uleb128 0x2
	.byte	0x8
	.byte	0x7
	.4byte	.LASF6
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF7
	.uleb128 0x4
	.byte	0x4
	.byte	0x7
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF8
	.uleb128 0x2
	.byte	0x4
	.byte	0x5
	.4byte	.LASF9
	.uleb128 0x2
	.byte	0x1
	.byte	0x2
	.4byte	.LASF10
	.uleb128 0x5
	.4byte	.LASF42
	.byte	0x4
	.byte	0x2
	.byte	0x4b
	.4byte	0x143
	.uleb128 0x6
	.4byte	.LASF11
	.sleb128 0
	.uleb128 0x6
	.4byte	.LASF12
	.sleb128 1
	.uleb128 0x6
	.4byte	.LASF13
	.sleb128 2
	.uleb128 0x6
	.4byte	.LASF14
	.sleb128 3
	.uleb128 0x6
	.4byte	.LASF15
	.sleb128 4
	.uleb128 0x6
	.4byte	.LASF16
	.sleb128 5
	.uleb128 0x6
	.4byte	.LASF17
	.sleb128 6
	.uleb128 0x6
	.4byte	.LASF18
	.sleb128 7
	.uleb128 0x6
	.4byte	.LASF19
	.sleb128 8
	.uleb128 0x6
	.4byte	.LASF20
	.sleb128 9
	.uleb128 0x6
	.4byte	.LASF21
	.sleb128 10
	.uleb128 0x6
	.4byte	.LASF22
	.sleb128 11
	.uleb128 0x6
	.4byte	.LASF23
	.sleb128 12
	.uleb128 0x6
	.4byte	.LASF24
	.sleb128 13
	.uleb128 0x6
	.4byte	.LASF25
	.sleb128 14
	.uleb128 0x6
	.4byte	.LASF26
	.sleb128 15
	.uleb128 0x6
	.4byte	.LASF27
	.sleb128 16
	.uleb128 0x6
	.4byte	.LASF28
	.sleb128 17
	.uleb128 0x6
	.4byte	.LASF29
	.sleb128 18
	.uleb128 0x6
	.4byte	.LASF30
	.sleb128 19
	.uleb128 0x6
	.4byte	.LASF31
	.sleb128 20
	.uleb128 0x6
	.4byte	.LASF32
	.sleb128 21
	.uleb128 0x6
	.4byte	.LASF33
	.sleb128 22
	.uleb128 0x6
	.4byte	.LASF34
	.sleb128 23
	.uleb128 0x6
	.4byte	.LASF35
	.sleb128 8
	.uleb128 0x6
	.4byte	.LASF36
	.sleb128 12
	.uleb128 0x6
	.4byte	.LASF37
	.sleb128 8
	.uleb128 0x6
	.4byte	.LASF38
	.sleb128 4
	.uleb128 0x6
	.4byte	.LASF39
	.sleb128 11
	.uleb128 0x6
	.4byte	.LASF40
	.sleb128 6
	.uleb128 0x6
	.4byte	.LASF41
	.sleb128 1
	.byte	0x0
	.uleb128 0x5
	.4byte	.LASF43
	.byte	0x4
	.byte	0x3
	.byte	0xbf
	.4byte	0x168
	.uleb128 0x6
	.4byte	.LASF44
	.sleb128 0
	.uleb128 0x6
	.4byte	.LASF45
	.sleb128 1
	.uleb128 0x6
	.4byte	.LASF46
	.sleb128 2
	.uleb128 0x6
	.4byte	.LASF47
	.sleb128 3
	.byte	0x0
	.uleb128 0x7
	.byte	0x1
	.ascii	"foo\000"
	.byte	0x1
	.byte	0xd
	.byte	0x1
	.4byte	.LFB0
	.4byte	.LFE0
	.4byte	.LLST0
	.byte	0x0
	.section	.debug_abbrev
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x25
	.uleb128 0xe
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1b
	.uleb128 0xe
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x10
	.uleb128 0x6
	.byte	0x0
	.byte	0x0
	.uleb128 0x2
	.uleb128 0x24
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.byte	0x0
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x24
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.byte	0x0
	.byte	0x0
	.uleb128 0x4
	.uleb128 0x24
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.byte	0x0
	.byte	0x0
	.uleb128 0x5
	.uleb128 0x4
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x6
	.uleb128 0x28
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1c
	.uleb128 0xd
	.byte	0x0
	.byte	0x0
	.uleb128 0x7
	.uleb128 0x2e
	.byte	0x0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.section	.debug_pubnames,"",%progbits
	.4byte	0x16
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x17e
	.4byte	0x168
	.ascii	"foo\000"
	.4byte	0x0
	.section	.debug_aranges,"",%progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0x0
	.2byte	0x0
	.2byte	0x0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	0x0
	.4byte	0x0
	.section	.debug_str,"MS",%progbits,1
.LASF28:
	.ascii	"PG_mappedtodisk\000"
.LASF50:
	.ascii	"/home/hanny/kernel/Lucifr\000"
.LASF22:
	.ascii	"PG_private\000"
.LASF2:
	.ascii	"short int\000"
.LASF37:
	.ascii	"PG_pinned\000"
.LASF45:
	.ascii	"ZONE_HIGHMEM\000"
.LASF29:
	.ascii	"PG_reclaim\000"
.LASF40:
	.ascii	"PG_slub_frozen\000"
.LASF36:
	.ascii	"PG_fscache\000"
.LASF21:
	.ascii	"PG_reserved\000"
.LASF11:
	.ascii	"PG_locked\000"
.LASF34:
	.ascii	"__NR_PAGEFLAGS\000"
.LASF31:
	.ascii	"PG_swapbacked\000"
.LASF30:
	.ascii	"PG_buddy\000"
.LASF5:
	.ascii	"long long int\000"
.LASF46:
	.ascii	"ZONE_MOVABLE\000"
.LASF9:
	.ascii	"long int\000"
.LASF47:
	.ascii	"__MAX_NR_ZONES\000"
.LASF44:
	.ascii	"ZONE_NORMAL\000"
.LASF20:
	.ascii	"PG_arch_1\000"
.LASF19:
	.ascii	"PG_owner_priv_1\000"
.LASF1:
	.ascii	"unsigned char\000"
.LASF14:
	.ascii	"PG_uptodate\000"
.LASF0:
	.ascii	"signed char\000"
.LASF6:
	.ascii	"long long unsigned int\000"
.LASF26:
	.ascii	"PG_tail\000"
.LASF39:
	.ascii	"PG_slob_free\000"
.LASF4:
	.ascii	"unsigned int\000"
.LASF13:
	.ascii	"PG_referenced\000"
.LASF15:
	.ascii	"PG_dirty\000"
.LASF41:
	.ascii	"PG_slub_debug\000"
.LASF3:
	.ascii	"short unsigned int\000"
.LASF35:
	.ascii	"PG_checked\000"
.LASF8:
	.ascii	"char\000"
.LASF42:
	.ascii	"pageflags\000"
.LASF10:
	.ascii	"_Bool\000"
.LASF16:
	.ascii	"PG_lru\000"
.LASF32:
	.ascii	"PG_unevictable\000"
.LASF7:
	.ascii	"long unsigned int\000"
.LASF27:
	.ascii	"PG_swapcache\000"
.LASF17:
	.ascii	"PG_active\000"
.LASF25:
	.ascii	"PG_head\000"
.LASF49:
	.ascii	"kernel/bounds.c\000"
.LASF43:
	.ascii	"zone_type\000"
.LASF23:
	.ascii	"PG_private_2\000"
.LASF18:
	.ascii	"PG_slab\000"
.LASF33:
	.ascii	"PG_mlocked\000"
.LASF12:
	.ascii	"PG_error\000"
.LASF24:
	.ascii	"PG_writeback\000"
.LASF48:
	.ascii	"GNU C 4.4.1\000"
.LASF38:
	.ascii	"PG_savepinned\000"
	.ident	"GCC: (Sourcery G++ Lite 2009q3-67) 4.4.1"
	.section	.note.GNU-stack,"",%progbits
