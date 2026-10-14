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
	.eabi_attribute	18, 4
	.eabi_attribute	14, 0
	.file	"spi_regression.18f8146eda39b5ed-cgu.0"
	.section	.text.spi_transfer,"ax",%progbits
	.globl	spi_transfer
	.p2align	2
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
	.save	{r8, r9, r10, r11}
	push.w	{r8, r9, r10, r11}
	.cfi_offset r11, -24
	.cfi_offset r10, -28
	.cfi_offset r9, -32
	.cfi_offset r8, -36
	.pad	#28
	sub	sp, #28
	ldrd	r10, r0, [r0]
	mov.w	r3, #15360
	movs	r6, #1
	movw	lr, #0
	str	r0, [sp, #4]
	movt	lr, #16386
	ldrb	r0, [r0]
	and.w	r3, r3, r0, lsl #6
	bfi	r0, r6, #4, #28
	orr.w	r3, r3, lr
	lsl.w	r0, r6, r0
	str	r0, [r3, #24]
	movs	r0, #2
	strd	r0, r1, [sp, #8]
	movs	r1, #0
	str	r0, [sp, #24]
	strd	r0, r2, [sp, #16]
.LBB0_1:
	cmp	r1, #20
	beq.w	.LBB0_129
	add	r2, sp, #8
	add.w	r3, r1, #20
	adds	r0, r2, r1
	ldr	r2, [r2, r1]
	mov	r1, r3
.LCPI0_0:
	tbh	[pc, r2, lsl #1]
.LJTI0_0:
	.short	(.LBB0_4-(.LCPI0_0+4))/2
	.short	(.LBB0_13-(.LCPI0_0+4))/2
	.short	(.LBB0_7-(.LCPI0_0+4))/2
	.short	(.LBB0_10-(.LCPI0_0+4))/2
	.short	(.LBB0_136-(.LCPI0_0+4))/2
	.p2align	1
.LBB0_4:
	ldrd	r12, r4, [r0, #4]
	ldr.w	r2, [r10, #4]
	ldrb.w	r3, [r10, #23]
	cbz	r3, .LBB0_6
	ldr	r0, [r2]
	movs	r5, #0
	ldr	r3, [r0]
	bic	r3, r3, #64
	str	r3, [r0]
	ldr	r0, [r2]
	ldr	r3, [r0]
	strb.w	r5, [r10, #23]
	bic	r3, r3, #2048
	str	r3, [r0]
.LBB0_6:
	ldr	r0, [r2]
	ldr	r3, [r0]
	orr	r3, r3, #64
	str	r3, [r0]
	ldr	r3, [r2]
	b	.LBB0_71
.LBB0_7:
	ldrd	r8, r2, [r0, #4]
	ldrd	r11, r6, [r0, #12]
	ldr.w	r3, [r10, #4]
	ldrb.w	r4, [r10, #23]
	cbz	r4, .LBB0_9
	ldr	r0, [r3]
	movs	r5, #0
	ldr	r4, [r0]
	bic	r4, r4, #64
	str	r4, [r0]
	ldr	r0, [r3]
	ldr	r4, [r0]
	strb.w	r5, [r10, #23]
	bic	r4, r4, #2048
	str	r4, [r0]
.LBB0_9:
	ldr	r0, [r3]
	ldr	r4, [r0]
	orr	r4, r4, #64
	str	r4, [r0]
	ldr	r4, [r3]
	b	.LBB0_17
.LBB0_10:
	ldrd	r12, r4, [r0, #4]
	ldr.w	r2, [r10, #4]
	ldrb.w	r3, [r10, #23]
	cbz	r3, .LBB0_12
	ldr	r0, [r2]
	movs	r5, #0
	ldr	r3, [r0]
	bic	r3, r3, #64
	str	r3, [r0]
	ldr	r0, [r2]
	ldr	r3, [r0]
	strb.w	r5, [r10, #23]
	bic	r3, r3, #2048
	str	r3, [r0]
.LBB0_12:
	ldr	r0, [r2]
	ldr	r3, [r0]
	orr	r3, r3, #64
	str	r3, [r0]
	ldr	r3, [r2]
	b	.LBB0_44
.LBB0_13:
	ldrd	r2, r6, [r0, #4]
	ldr.w	r3, [r10, #4]
	ldrb.w	r5, [r10, #23]
	cbz	r5, .LBB0_15
	ldr	r0, [r3]
	movs	r4, #0
	ldr	r5, [r0]
	bic	r5, r5, #64
	str	r5, [r0]
	ldr	r0, [r3]
	ldr	r5, [r0]
	strb.w	r4, [r10, #23]
	bic	r5, r5, #2048
	str	r5, [r0]
.LBB0_15:
	ldr	r0, [r3]
	ldr	r5, [r0]
	orr	r5, r5, #64
	str	r5, [r0]
	ldr	r5, [r3]
	b	.LBB0_97
.LBB0_16:
	ldr	r0, [r4, #12]
.LBB0_17:
	ldr	r0, [r4, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r4, #12]
	ldrne	r0, [r4, #8]
	lslsne.w	r0, r0, #31
	beq	.LBB0_19
	ldr	r0, [r4, #12]
	ldr	r0, [r4, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r4, #12]
	ldrne	r0, [r4, #8]
	lslsne.w	r0, r0, #31
	bne	.LBB0_16
.LBB0_19:
	cmp	r2, r6
	mov	r9, r6
	dmb	sy
	it	hi
	movhi	r9, r2
	cmp.w	r9, #0
	beq.w	.LBB0_1
	add.w	r5, r8, r2
	ldr	r2, [r3]
	add.w	r4, r11, r6
.LBB0_21:
	ldr	r0, [r2, #8]
	lsls	r3, r0, #25
	bmi.w	.LBB0_117
	lsls	r3, r0, #23
	bmi.w	.LBB0_118
	lsls	r3, r0, #26
	bmi.w	.LBB0_119
	lsls	r3, r0, #27
	bmi.w	.LBB0_120
	lsls	r0, r0, #30
	bpl	.LBB0_21
	movs	r3, #0
	mov.w	r12, #1
	movs	r6, #1
	mov.w	lr, #0
	b	.LBB0_28
.LBB0_27:
	cmp	lr, r9
	mov.w	r12, #0
	it	lo
	movlo.w	r12, #1
	cmp	r3, r9
	mov.w	r6, #0
	ite	lo
	movlo	r6, #1
	cmphs	lr, r9
	bhs.w	.LBB0_114
.LBB0_28:
	lsls	r0, r6, #31
	beq	.LBB0_36
	cmp	r3, lr
	bne	.LBB0_36
	ldr	r0, [r2, #8]
	lsls	r6, r0, #25
	bmi.w	.LBB0_130
	lsls	r6, r0, #23
	bmi.w	.LBB0_131
	lsls	r6, r0, #26
	bmi.w	.LBB0_132
	lsls	r6, r0, #27
	bmi.w	.LBB0_133
	lsls	r0, r0, #30
	bpl	.LBB0_36
	cmp	r11, r4
	itee	ne
	ldrbne	r0, [r11], #1
	moveq	r0, #0
	moveq	r11, r4
	strb	r0, [r2, #12]
	adds	r3, #1
.LBB0_36:
	cmp	lr, r3
	mov.w	r0, #0
	it	lo
	movlo	r0, #1
	tst.w	r12, r0
	beq	.LBB0_27
	ldr	r0, [r2, #8]
	lsls	r6, r0, #25
	bmi.w	.LBB0_130
	lsls	r6, r0, #23
	bmi.w	.LBB0_131
	lsls	r6, r0, #26
	bmi.w	.LBB0_132
	lsls	r6, r0, #27
	bmi.w	.LBB0_133
	lsls	r0, r0, #31
	beq	.LBB0_27
	cmp	r8, r5
	ldrb	r0, [r2, #12]
	ite	eq
	moveq	r8, r5
	strbne	r0, [r8], #1
	add.w	lr, lr, #1
	b	.LBB0_27
.LBB0_43:
	ldr	r0, [r3, #12]
.LBB0_44:
	ldr	r0, [r3, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r3, #12]
	ldrne	r0, [r3, #8]
	lslsne.w	r0, r0, #31
	beq	.LBB0_46
	ldr	r0, [r3, #12]
	ldr	r0, [r3, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r3, #12]
	ldrne	r0, [r3, #8]
	lslsne.w	r0, r0, #31
	bne	.LBB0_43
.LBB0_46:
	cmp	r4, #0
	dmb	sy
	beq.w	.LBB0_1
	ldr	r2, [r2]
	mov	r8, lr
	add.w	lr, r12, r4
.LBB0_48:
	ldr	r0, [r2, #8]
	lsls	r3, r0, #25
	bmi.w	.LBB0_122
	lsls	r3, r0, #23
	bmi.w	.LBB0_124
	lsls	r3, r0, #26
	bmi.w	.LBB0_126
	lsls	r3, r0, #27
	bmi.w	.LBB0_128
	lsls	r0, r0, #30
	bpl	.LBB0_48
	mov.w	r9, #0
	movs	r5, #1
	movs	r3, #1
	mov	r11, r12
	movs	r6, #0
	b	.LBB0_55
.LBB0_54:
	cmp	r6, r4
	mov.w	r5, #0
	it	lo
	movlo	r5, #1
	cmp	r9, r4
	mov.w	r3, #0
	ite	lo
	movlo	r3, #1
	cmphs	r6, r4
	bhs.w	.LBB0_115
.LBB0_55:
	lsls	r0, r3, #31
	beq	.LBB0_63
	cmp	r9, r6
	bne	.LBB0_63
	ldr	r0, [r2, #8]
	lsls	r3, r0, #25
	bmi.w	.LBB0_122
	lsls	r3, r0, #23
	bmi.w	.LBB0_124
	lsls	r3, r0, #26
	bmi.w	.LBB0_126
	lsls	r3, r0, #27
	bmi.w	.LBB0_128
	lsls	r0, r0, #30
	bpl	.LBB0_63
	cmp	r11, lr
	itee	ne
	ldrbne	r0, [r11], #1
	moveq	r0, #0
	moveq	r11, lr
	strb	r0, [r2, #12]
	add.w	r9, r9, #1
.LBB0_63:
	cmp	r6, r9
	mov.w	r0, #0
	it	lo
	movlo	r0, #1
	tst	r5, r0
	beq	.LBB0_54
	ldr	r0, [r2, #8]
	lsls	r3, r0, #25
	bmi.w	.LBB0_122
	lsls	r3, r0, #23
	bmi.w	.LBB0_124
	lsls	r3, r0, #26
	bmi.w	.LBB0_126
	lsls	r3, r0, #27
	bmi.w	.LBB0_128
	lsls	r0, r0, #31
	beq	.LBB0_54
	cmp	r12, lr
	ldrb	r0, [r2, #12]
	ite	eq
	moveq	r12, lr
	strbne	r0, [r12], #1
	adds	r6, #1
	b	.LBB0_54
.LBB0_70:
	ldr	r0, [r3, #12]
.LBB0_71:
	ldr	r0, [r3, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r3, #12]
	ldrne	r0, [r3, #8]
	lslsne.w	r0, r0, #31
	beq	.LBB0_73
	ldr	r0, [r3, #12]
	ldr	r0, [r3, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r3, #12]
	ldrne	r0, [r3, #8]
	lslsne.w	r0, r0, #31
	bne	.LBB0_70
.LBB0_73:
	cmp	r4, #0
	dmb	sy
	beq.w	.LBB0_1
	ldr	r2, [r2]
	mov	r9, lr
	add.w	lr, r12, r4
.LBB0_75:
	ldr	r0, [r2, #8]
	lsls	r3, r0, #25
	bmi.w	.LBB0_121
	lsls	r3, r0, #23
	bmi.w	.LBB0_123
	lsls	r3, r0, #26
	bmi.w	.LBB0_125
	lsls	r3, r0, #27
	bmi.w	.LBB0_127
	lsls	r0, r0, #30
	bpl	.LBB0_75
	mov.w	r8, #0
	movs	r5, #1
	movs	r0, #1
	movs	r3, #0
	b	.LBB0_82
.LBB0_81:
	cmp	r3, r4
	mov.w	r5, #0
	it	lo
	movlo	r5, #1
	cmp	r8, r4
	mov.w	r0, #0
	ite	lo
	movlo	r0, #1
	cmphs	r3, r4
	bhs	.LBB0_116
.LBB0_82:
	lsls	r0, r0, #31
	beq	.LBB0_89
	cmp	r8, r3
	bne	.LBB0_89
	ldr	r0, [r2, #8]
	lsls	r6, r0, #25
	bmi	.LBB0_121
	lsls	r6, r0, #23
	bmi	.LBB0_123
	lsls	r6, r0, #26
	bmi	.LBB0_125
	lsls	r6, r0, #27
	bmi	.LBB0_127
	lsls	r0, r0, #30
	ittt	mi
	movmi	r0, #0
	strbmi	r0, [r2, #12]
	addmi.w	r8, r8, #1
.LBB0_89:
	cmp	r3, r8
	mov.w	r0, #0
	it	lo
	movlo	r0, #1
	tst	r5, r0
	beq	.LBB0_81
	ldr	r0, [r2, #8]
	lsls	r5, r0, #25
	bmi	.LBB0_121
	lsls	r5, r0, #23
	bmi	.LBB0_123
	lsls	r5, r0, #26
	bmi	.LBB0_125
	lsls	r5, r0, #27
	bmi	.LBB0_127
	lsls	r0, r0, #31
	beq	.LBB0_81
	cmp	r12, lr
	ldrb	r0, [r2, #12]
	ite	eq
	moveq	r12, lr
	strbne	r0, [r12], #1
	adds	r3, #1
	b	.LBB0_81
.LBB0_96:
	ldr	r0, [r5, #12]
.LBB0_97:
	ldr	r0, [r5, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r5, #12]
	ldrne	r0, [r5, #8]
	lslsne.w	r0, r0, #31
	beq	.LBB0_99
	ldr	r0, [r5, #12]
	ldr	r0, [r5, #8]
	lsls	r0, r0, #31
	ittt	ne
	ldrne	r0, [r5, #12]
	ldrne	r0, [r5, #8]
	lslsne.w	r0, r0, #31
	bne	.LBB0_96
.LBB0_99:
	cmp	r6, #0
	dmb	sy
	beq.w	.LBB0_1
	add.w	r12, r2, r6
.LBB0_101:
	ldrb	r0, [r2]
	ldr	r5, [r3]
.LBB0_102:
	ldr	r4, [r5, #8]
	lsls	r6, r4, #25
	bmi	.LBB0_117
	lsls	r6, r4, #23
	bmi	.LBB0_118
	lsls	r6, r4, #26
	bmi	.LBB0_119
	lsls	r6, r4, #27
	bmi	.LBB0_120
	lsls	r6, r4, #30
	bpl	.LBB0_102
	strb	r0, [r5, #12]
.LBB0_108:
	ldr	r0, [r5, #8]
	lsls	r6, r0, #25
	bmi	.LBB0_117
	lsls	r6, r0, #23
	bmi	.LBB0_118
	lsls	r6, r0, #26
	bmi	.LBB0_119
	lsls	r6, r0, #27
	bmi	.LBB0_120
	lsls	r0, r0, #31
	beq	.LBB0_108
	adds	r2, #1
	ldrb	r0, [r5, #12]
	cmp	r2, r12
	bne	.LBB0_101
	b	.LBB0_1
.LBB0_114:
	movw	lr, #0
	movt	lr, #16386
	b	.LBB0_1
.LBB0_115:
	mov	lr, r8
	b	.LBB0_1
.LBB0_116:
	mov	lr, r9
	b	.LBB0_1
.LBB0_117:
	movs	r0, #3
	b	.LBB0_135
.LBB0_118:
	movs	r0, #0
	b	.LBB0_135
.LBB0_119:
	movs	r0, #2
	b	.LBB0_135
.LBB0_120:
	movs	r0, #1
	b	.LBB0_135
.LBB0_121:
	movs	r0, #3
	mov	lr, r9
	b	.LBB0_135
.LBB0_122:
	movs	r0, #3
	mov	lr, r8
	b	.LBB0_135
.LBB0_123:
	movs	r0, #0
	mov	lr, r9
	b	.LBB0_135
.LBB0_124:
	movs	r0, #0
	mov	lr, r8
	b	.LBB0_135
.LBB0_125:
	movs	r0, #2
	mov	lr, r9
	b	.LBB0_135
.LBB0_126:
	movs	r0, #2
	mov	lr, r8
	b	.LBB0_135
.LBB0_127:
	movs	r0, #1
	mov	lr, r9
	b	.LBB0_135
.LBB0_128:
	movs	r0, #1
	mov	lr, r8
	b	.LBB0_135
.LBB0_129:
	movs	r0, #255
	b	.LBB0_135
.LBB0_130:
	movs	r0, #3
	b	.LBB0_134
.LBB0_131:
	movs	r0, #0
	b	.LBB0_134
.LBB0_132:
	movs	r0, #2
	b	.LBB0_134
.LBB0_133:
	movs	r0, #1
.LBB0_134:
	movw	lr, #0
	movt	lr, #16386
.LBB0_135:
	ldr	r1, [sp, #4]
	movs	r3, #1
	ldrb	r1, [r1]
	and	r2, r1, #240
	and	r1, r1, #15
	orr.w	r2, lr, r2, lsl #6
	lsl.w	r1, r3, r1
	str	r1, [r2, #24]
	add	sp, #28
	pop.w	{r8, r9, r10, r11}
	pop	{r4, r5, r6, r7, pc}
.LBB0_136:
	bl	_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic
.Lfunc_end0:
	.size	spi_transfer, .Lfunc_end0-spi_transfer
	.cfi_endproc
	.cantunwind
	.fnend

	.section	.text.unlikely._RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt,"ax",%progbits
	.p2align	1
	.type	_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt,%function
	.code	16
	.thumb_func
_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt:
.Lfunc_begin1:
	.file	1 "/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11" "library/core/src/panicking.rs"
	.loc	1 60 0
	.fnstart
	.cfi_startproc
	.save	{r7, lr}
	push	{r7, lr}
	.cfi_def_cfa_offset 8
	.cfi_offset lr, -4
	.cfi_offset r7, -8
	.setfp	r7, sp
	mov	r7, sp
	.cfi_def_cfa_register r7
.LBB1_1:
.Ltmp0:
	.loc	1 80 14 prologue_end
	yield
	yield
	yield
	yield
	b	.LBB1_1
.Ltmp1:
.Lfunc_end1:
	.size	_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt, .Lfunc_end1-_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt
	.cfi_endproc
	.cantunwind
	.fnend

	.section	.text.unlikely._RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic,"ax",%progbits
	.p2align	1
	.type	_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic,%function
	.code	16
	.thumb_func
_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic:
.Lfunc_begin2:
	.fnstart
	.cfi_startproc
	.save	{r7, lr}
	push	{r7, lr}
	.cfi_def_cfa_offset 8
	.cfi_offset lr, -4
	.cfi_offset r7, -8
	.setfp	r7, sp
	mov	r7, sp
	.cfi_def_cfa_register r7
	bl	_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt
.Lfunc_end2:
	.size	_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic, .Lfunc_end2-_RNvNtCsgYNjhtQEXp_16embedded_hal_bus3spi14no_delay_panic
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

	.section	.debug_abbrev,"",%progbits
	.byte	1
	.byte	17
	.byte	1
	.byte	37
	.byte	14
	.byte	19
	.byte	5
	.byte	3
	.byte	14
	.byte	16
	.byte	23
	.byte	27
	.byte	14
	.byte	17
	.byte	1
	.byte	18
	.byte	6
	.byte	0
	.byte	0
	.byte	2
	.byte	57
	.byte	1
	.byte	3
	.byte	14
	.byte	0
	.byte	0
	.byte	3
	.byte	46
	.byte	0
	.byte	17
	.byte	1
	.byte	18
	.byte	6
	.byte	64
	.byte	24
	.byte	110
	.byte	14
	.byte	3
	.byte	14
	.byte	58
	.byte	11
	.byte	59
	.byte	11
	.byte	54
	.byte	11
	.byte	63
	.byte	25
	.ascii	"\207\001"
	.byte	25
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_info,"",%progbits
.Lcu_begin0:
	.long	.Ldebug_info_end0-.Ldebug_info_start0
.Ldebug_info_start0:
	.short	4
	.long	.debug_abbrev
	.byte	4
	.byte	1
	.long	.Linfo_string0
	.short	28
	.long	.Linfo_string1
	.long	.Lline_table_start0
	.long	.Linfo_string2
	.long	.Lfunc_begin1
	.long	.Lfunc_end1-.Lfunc_begin1
	.byte	2
	.long	.Linfo_string3
	.byte	2
	.long	.Linfo_string4
	.byte	3
	.long	.Lfunc_begin1
	.long	.Lfunc_end1-.Lfunc_begin1
	.byte	1
	.byte	87
	.long	.Linfo_string5
	.long	.Linfo_string6
	.byte	1
	.byte	60
	.byte	3


	.byte	0
	.byte	0
	.byte	0
.Ldebug_info_end0:
	.section	.text.unlikely._RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt,"ax",%progbits
.Lsec_end0:
	.section	.debug_aranges,"",%progbits
	.long	28
	.short	2
	.long	.Lcu_begin0
	.byte	4
	.byte	0
	.zero	4,255
	.long	.Lfunc_begin1
	.long	.Lsec_end0-.Lfunc_begin1
	.long	0
	.long	0
	.section	.debug_str,"MS",%progbits,1
.Linfo_string0:
	.asciz	"clang LLVM (rustc version 1.101.0-nightly (822182db4 2026-10-13))"
.Linfo_string1:
	.asciz	"/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11/library/core/src/lib.rs/@/core.14fadb22da8e1282-cgu.0"
.Linfo_string2:
	.asciz	"/rustc/822182db4c4a1827256881d8c7aa685f0bcf2b11"
.Linfo_string3:
	.asciz	"core"
.Linfo_string4:
	.asciz	"panicking"
.Linfo_string5:
	.asciz	"_RNvNtCs1NFSOanQ2Tm_4core9panicking9panic_fmt"
.Linfo_string6:
	.asciz	"panic_fmt"
	.ident	"rustc version 1.101.0-nightly (822182db4 2026-10-13)"
	.section	".note.GNU-stack","",%progbits
	.eabi_attribute	30, 2
	.section	.debug_line,"",%progbits
.Lline_table_start0:
