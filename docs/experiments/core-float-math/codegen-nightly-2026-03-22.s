	.syntax	unified
	.eabi_attribute	67, "2.09"
	.eabi_attribute	6, 13
	.eabi_attribute	7, 77
	.eabi_attribute	8, 0
	.eabi_attribute	9, 2
	.fpu	fpv4-sp-d16
	.eabi_attribute	27, 1
	.eabi_attribute	36, 1
	.eabi_attribute	34, 1
	.eabi_attribute	17, 1
	.eabi_attribute	20, 1
	.eabi_attribute	21, 1
	.eabi_attribute	23, 3
	.eabi_attribute	24, 1
	.eabi_attribute	25, 1
	.eabi_attribute	28, 1
	.eabi_attribute	38, 1
	.eabi_attribute	14, 0
	.file	"codegen_probe.1a814217aa1fca7b-cgu.0"
	.section	.text.asm_f32_to_q31,"ax",%progbits
	.globl	asm_f32_to_q31
	.p2align	1
	.type	asm_f32_to_q31,%function
	.code	16
	.thumb_func
asm_f32_to_q31:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	@APP
	vcvt.s32.f32	s0, s0, #31
	vmov	r0, s0
	@NO_APP
	pop	{r7, pc}
.Lfunc_end0:
	.size	asm_f32_to_q31, .Lfunc_end0-asm_f32_to_q31
	.cantunwind
	.fnend

	.section	.text.asm_q31_to_f32,"ax",%progbits
	.globl	asm_q31_to_f32
	.p2align	1
	.type	asm_q31_to_f32,%function
	.code	16
	.thumb_func
asm_q31_to_f32:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	@APP
	vmov	s0, r0
	vcvt.f32.s32	s0, s0, #31
	@NO_APP
	pop	{r7, pc}
.Lfunc_end1:
	.size	asm_q31_to_f32, .Lfunc_end1-asm_q31_to_f32
	.cantunwind
	.fnend

	.section	.text.asm_sqrt,"ax",%progbits
	.globl	asm_sqrt
	.p2align	1
	.type	asm_sqrt,%function
	.code	16
	.thumb_func
asm_sqrt:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	@APP
	vsqrt.f32	s2, s0
	@NO_APP
	vmov.f32	s0, s2
	pop	{r7, pc}
.Lfunc_end2:
	.size	asm_sqrt, .Lfunc_end2-asm_sqrt
	.cantunwind
	.fnend

	.section	.text.core_floor,"ax",%progbits
	.globl	core_floor
	.p2align	1
	.type	core_floor,%function
	.code	16
	.thumb_func
core_floor:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	pop.w	{r7, lr}
	b	floorf
.Lfunc_end3:
	.size	core_floor, .Lfunc_end3-core_floor
	.cantunwind
	.fnend

	.section	.text.core_mod,"ax",%progbits
	.globl	core_mod
	.p2align	1
	.type	core_mod,%function
	.code	16
	.thumb_func
core_mod:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	pop.w	{r7, lr}
	b	fmodf
.Lfunc_end4:
	.size	core_mod, .Lfunc_end4-core_mod
	.cantunwind
	.fnend

	.section	.text.core_round,"ax",%progbits
	.globl	core_round
	.p2align	1
	.type	core_round,%function
	.code	16
	.thumb_func
core_round:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	pop.w	{r7, lr}
	b	roundf
.Lfunc_end5:
	.size	core_round, .Lfunc_end5-core_round
	.cantunwind
	.fnend

	.section	.text.core_sqrt,"ax",%progbits
	.globl	core_sqrt
	.p2align	1
	.type	core_sqrt,%function
	.code	16
	.thumb_func
core_sqrt:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	vsqrt.f32	s0, s0
	pop	{r7, pc}
.Lfunc_end6:
	.size	core_sqrt, .Lfunc_end6-core_sqrt
	.cantunwind
	.fnend

	.section	.text.rust_f32_to_q31,"ax",%progbits
	.globl	rust_f32_to_q31
	.p2align	2
	.type	rust_f32_to_q31,%function
	.code	16
	.thumb_func
rust_f32_to_q31:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	vldr	s2, .LCPI7_0
	vmul.f32	s0, s0, s2
	vcvt.s32.f32	s0, s0
	vmov	r0, s0
	pop	{r7, pc}
	.p2align	2
.LCPI7_0:
	.long	0x4f000000
.Lfunc_end7:
	.size	rust_f32_to_q31, .Lfunc_end7-rust_f32_to_q31
	.cantunwind
	.fnend

	.section	.text.rust_q31_to_f32,"ax",%progbits
	.globl	rust_q31_to_f32
	.p2align	2
	.type	rust_q31_to_f32,%function
	.code	16
	.thumb_func
rust_q31_to_f32:
	.fnstart
	.save	{r7, lr}
	push	{r7, lr}
	.setfp	r7, sp
	mov	r7, sp
	vmov	s0, r0
	vldr	s2, .LCPI8_0
	vcvt.f32.s32	s0, s0
	vmul.f32	s0, s0, s2
	pop	{r7, pc}
	.p2align	2
.LCPI8_0:
	.long	0x30000000
.Lfunc_end8:
	.size	rust_q31_to_f32, .Lfunc_end8-rust_q31_to_f32
	.cantunwind
	.fnend

	.ident	"rustc version 1.96.0-nightly (20f19f461 2026-03-21)"
	.section	".note.GNU-stack","",%progbits
	.eabi_attribute	30, 2
