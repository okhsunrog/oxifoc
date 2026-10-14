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
	.file	"spi_regression.fd9826c05a46b782-cgu.0"
	.section	.text.spi_transfer,"ax",%progbits
	.globl	spi_transfer
	.p2align	1
	.type	spi_transfer,%function
	.code	16
	.thumb_func
spi_transfer:
.Lfunc_begin0:
	.fnstart
	.cfi_sections .debug_frame
	.cfi_startproc
	.save	{r4, r5, r6, r7, lr}
	push	{r4, r5, r6, r7, lr}
	.cfi_def_cfa_offset 20
	.cfi_offset lr, -4
	.cfi_offset r7, -8
	.cfi_offset r6, -12
	.cfi_offset r5, -16
	.cfi_offset r4, -20
	.setfp	r7, sp, #12
	add	r7, sp, #12
	.cfi_def_cfa r7, 8
	.save	{r8, r9, r10}
	push.w	{r8, r9, r10}
	.cfi_offset r10, -24
	.cfi_offset r9, -28
	.cfi_offset r8, -32
	ldrd	r0, r9, [r0]
	mov.w	r6, #15360
	ldrb.w	r3, [r9]
	movs	r5, #1
	movw	r12, #0
	movt	r12, #16386
	and.w	r6, r6, r3, lsl #6
	bfi	r3, r5, #4, #28
	orr.w	r6, r6, r12
	lsl.w	r3, r5, r3
	str	r3, [r6, #24]
	ldr	r3, [r0, #4]
	ldrb	r6, [r0, #23]
	cbz	r6, .LBB0_2
	ldr	r6, [r3]
	ldr	r5, [r6]
	bic	r5, r5, #64
	str	r5, [r6]
	ldr	r6, [r3]
	movs	r5, #0
	ldr	r4, [r6]
	strb	r5, [r0, #23]
	bic	r5, r4, #2048
	str	r5, [r6]
.LBB0_2:
	ldr	r6, [r3]
	ldr	r5, [r6]
	orr	r5, r5, #64
	str	r5, [r6]
	ldr	r3, [r3]
	b	.LBB0_4
.LBB0_3:
	ldr	r6, [r3, #12]
.LBB0_4:
	ldr	r6, [r3, #8]
	lsls	r6, r6, #31
	ittt	ne
	ldrne	r6, [r3, #12]
	ldrne	r6, [r3, #8]
	lslsne.w	r6, r6, #31
	beq	.LBB0_6
	ldr	r6, [r3, #12]
	ldr	r6, [r3, #8]
	lsls	r6, r6, #31
	ittt	ne
	ldrne	r6, [r3, #12]
	ldrne	r6, [r3, #8]
	lslsne.w	r6, r6, #31
	bne	.LBB0_3
.LBB0_6:
	dmb	sy
	ldr	r0, [r0, #4]
	ldr	r0, [r0]
.LBB0_7:
	ldr	r3, [r0, #8]
	lsls	r6, r3, #25
	bmi	.LBB0_34
	lsls	r6, r3, #23
	bmi	.LBB0_35
	lsls	r6, r3, #26
	bmi	.LBB0_36
	lsls	r6, r3, #27
	bmi	.LBB0_37
	lsls	r3, r3, #30
	bpl	.LBB0_7
	mov.w	lr, #0
	movs	r5, #1
	movs	r3, #1
	mov.w	r8, #0
	mov.w	r10, #0
	movs	r6, #0
	b	.LBB0_15
.LBB0_13:
	mov.w	lr, #2
	ldrb	r3, [r0, #12]
	adds	r6, #1
.LBB0_14:
	cmp	r6, #2
	mov.w	r5, #0
	it	lo
	movlo	r5, #1
	cmp.w	r10, #2
	mov.w	r3, #0
	ite	lo
	movlo	r3, #1
	cmphs	r6, #2
	bhs	.LBB0_39
.LBB0_15:
	lsls	r3, r3, #31
	beq	.LBB0_26
	cmp	r10, r6
	bne	.LBB0_26
	ldr	r3, [r0, #8]
	lsls	r4, r3, #25
	bmi	.LBB0_34
	lsls	r4, r3, #23
	bmi	.LBB0_35
	lsls	r4, r3, #26
	bmi	.LBB0_36
	lsls	r4, r3, #27
	bmi	.LBB0_37
	lsls	r3, r3, #30
	bpl	.LBB0_26
	cmp.w	r8, #2
	bne	.LBB0_24
	mov.w	r8, #2
	movs	r3, #0
	b	.LBB0_25
.LBB0_24:
	ldrb.w	r3, [r2, r8]
	add.w	r8, r8, #1
.LBB0_25:
	strb	r3, [r0, #12]
	add.w	r10, r10, #1
.LBB0_26:
	cmp	r6, r10
	mov.w	r3, #0
	it	lo
	movlo	r3, #1
	tst	r5, r3
	beq	.LBB0_14
	ldr	r3, [r0, #8]
	lsls	r4, r3, #25
	bmi	.LBB0_34
	lsls	r4, r3, #23
	bmi	.LBB0_35
	lsls	r4, r3, #26
	bmi	.LBB0_36
	lsls	r4, r3, #27
	bmi	.LBB0_37
	lsls	r3, r3, #31
	beq	.LBB0_14
	cmp.w	lr, #2
	beq	.LBB0_13
	ldrb	r3, [r0, #12]
	adds	r6, #1
	strb.w	r3, [r1, lr]
	add.w	lr, lr, #1
	b	.LBB0_14
.LBB0_34:
	movs	r0, #3
	b	.LBB0_38
.LBB0_35:
	movs	r0, #0
	b	.LBB0_38
.LBB0_36:
	movs	r0, #2
	b	.LBB0_38
.LBB0_37:
	movs	r0, #1
.LBB0_38:
	ldrb.w	r1, [r9]
	movs	r3, #1
	and	r2, r1, #240
	and	r1, r1, #15
	orr.w	r2, r12, r2, lsl #6
	lsl.w	r1, r3, r1
	str	r1, [r2, #24]
	pop.w	{r8, r9, r10}
	pop	{r4, r5, r6, r7, pc}
.LBB0_39:
	movs	r0, #255
	b	.LBB0_38
.Lfunc_end0:
	.size	spi_transfer, .Lfunc_end0-spi_transfer
	.cfi_endproc
	.cantunwind
	.fnend

	.type	CORE_PERIPHERALS,%object
	.section	.rodata.CORE_PERIPHERALS,"a",%progbits
	.globl	CORE_PERIPHERALS
CORE_PERIPHERALS:
	.size	CORE_PERIPHERALS, 0

	.type	_EMBASSY_DEVICE_PERIPHERALS,%object
	.section	.bss._EMBASSY_DEVICE_PERIPHERALS,"aw",%nobits
	.globl	_EMBASSY_DEVICE_PERIPHERALS
_EMBASSY_DEVICE_PERIPHERALS:
	.zero	1
	.size	_EMBASSY_DEVICE_PERIPHERALS, 1

	.ident	"rustc version 1.99.0-nightly (09ee43b2d 2026-07-27)"
	.section	".note.GNU-stack","",%progbits
	.eabi_attribute	30, 2
