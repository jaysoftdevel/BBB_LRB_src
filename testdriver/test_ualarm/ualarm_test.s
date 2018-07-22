	.file	"ualarm_test.c"
	.data
	.align 4
	.type	nextStep, @object
	.size	nextStep, 4
nextStep:
	.long	1
	.align 4
	.type	nextDisp, @object
	.size	nextDisp, 4
nextDisp:
	.long	1
	.align 4
	.type	delayStep, @object
	.size	delayStep, 4
delayStep:
	.long	15000
	.align 4
	.type	delayDisp, @object
	.size	delayDisp, 4
delayDisp:
	.long	100000
	.align 4
	.type	heartBeat, @object
	.size	heartBeat, 4
heartBeat:
	.long	500
	.local	beatStep
	.comm	beatStep,4,4
	.local	beatDisp
	.comm	beatDisp,4,4
	.local	gStep
	.comm	gStep,4,4
	.local	gDisp
	.comm	gDisp,4,4
	.local	rval
	.comm	rval,4,4
	.comm	tidStep,4,4
	.comm	tidDisp,4,4
	.text
	.globl	actHB_handler
	.type	actHB_handler, @function
actHB_handler:
.LFB2:
	.cfi_startproc
	pushl	%ebp
	.cfi_def_cfa_offset 8
	.cfi_offset 5, -8
	movl	%esp, %ebp
	.cfi_def_cfa_register 5
	movl	beatStep, %edx
	movl	heartBeat, %eax
	addl	%edx, %eax
	movl	%eax, beatStep
	movl	beatDisp, %edx
	movl	heartBeat, %eax
	addl	%edx, %eax
	movl	%eax, beatDisp
	movl	beatStep, %edx
	movl	delayStep, %eax
	cmpl	%eax, %edx
	jb	.L2
	movl	$1, nextStep
	movl	$0, beatStep
.L2:
	movl	beatDisp, %edx
	movl	delayDisp, %eax
	cmpl	%eax, %edx
	jb	.L5
	movl	$1, nextDisp
	movl	$0, beatDisp
	nop
.L5:
	nop
	popl	%ebp
	.cfi_restore 5
	.cfi_def_cfa 4, 4
	ret
	.cfi_endproc
.LFE2:
	.size	actHB_handler, .-actHB_handler
	.globl	threadStep
	.type	threadStep, @function
threadStep:
.LFB3:
	.cfi_startproc
	pushl	%ebp
	.cfi_def_cfa_offset 8
	.cfi_offset 5, -8
	movl	%esp, %ebp
	.cfi_def_cfa_register 5
	subl	$8, %esp
.L11:
	movl	nextStep, %eax
	testl	%eax, %eax
	je	.L7
	movl	gStep, %eax
	addl	$1, %eax
	movl	%eax, gStep
	movl	$0, nextStep
.L7:
	movl	gStep, %eax
	cmpl	$99999, %eax
	jbe	.L9
	subl	$12, %esp
	pushl	$rval
	call	pthread_exit
.L10:
	subl	$12, %esp
	pushl	$5
	call	usleep
	addl	$16, %esp
.L9:
	movl	nextStep, %eax
	testl	%eax, %eax
	je	.L10
	jmp	.L11
	.cfi_endproc
.LFE3:
	.size	threadStep, .-threadStep
	.globl	threadDisp
	.type	threadDisp, @function
threadDisp:
.LFB4:
	.cfi_startproc
	pushl	%ebp
	.cfi_def_cfa_offset 8
	.cfi_offset 5, -8
	movl	%esp, %ebp
	.cfi_def_cfa_register 5
	subl	$8, %esp
.L17:
	movl	nextDisp, %eax
	testl	%eax, %eax
	je	.L13
	movl	gDisp, %eax
	addl	$1, %eax
	movl	%eax, gDisp
	movl	$0, nextDisp
.L13:
	movl	gDisp, %eax
	cmpl	$14999, %eax
	jbe	.L15
	subl	$12, %esp
	pushl	$rval
	call	pthread_exit
.L16:
	subl	$12, %esp
	pushl	$5
	call	usleep
	addl	$16, %esp
.L15:
	movl	nextDisp, %eax
	testl	%eax, %eax
	je	.L16
	jmp	.L17
	.cfi_endproc
.LFE4:
	.size	threadDisp, .-threadDisp
	.section	.rodata
.LC0:
	.string	"error creating thread"
.LC1:
	.string	"delay_ms error: %s\n"
.LC2:
	.string	"thread name: %s len: %i\n"
.LC3:
	.string	"go"
.LC5:
	.string	"done...: %.3lfms\n"
	.text
	.globl	main
	.type	main, @function
main:
.LFB5:
	.cfi_startproc
	leal	4(%esp), %ecx
	.cfi_def_cfa 1, 0
	andl	$-16, %esp
	pushl	-4(%ecx)
	pushl	%ebp
	.cfi_escape 0x10,0x5,0x2,0x75,0
	movl	%esp, %ebp
	pushl	%ecx
	.cfi_escape 0xf,0x3,0x75,0x7c,0x6
	subl	$196, %esp
	movl	$0, -12(%ebp)
	pushl	$0
	pushl	$threadStep
	pushl	$0
	pushl	$tidStep
	call	pthread_create
	addl	$16, %esp
	testl	%eax, %eax
	je	.L19
	subl	$12, %esp
	pushl	$.LC0
	call	puts
	addl	$16, %esp
	movl	$-1, %eax
	jmp	.L26
.L19:
	movl	-16(%ebp), %eax
	movzbl	(%eax), %eax
	movsbl	%al, %eax
	subl	$4, %esp
	pushl	-20(%ebp)
	pushl	%eax
	pushl	$threadStep
	call	pthread_getname_np
	addl	$16, %esp
	testl	%eax, %eax
	je	.L21
	call	__errno_location
	movl	(%eax), %eax
	subl	$12, %esp
	pushl	%eax
	call	strerror
	addl	$16, %esp
	subl	$8, %esp
	pushl	%eax
	pushl	$.LC1
	call	printf
	addl	$16, %esp
	movl	$-1, %eax
	jmp	.L26
.L21:
	subl	$4, %esp
	pushl	-20(%ebp)
	pushl	-16(%ebp)
	pushl	$.LC2
	call	printf
	addl	$16, %esp
	pushl	$0
	pushl	$threadDisp
	pushl	$0
	pushl	$tidDisp
	call	pthread_create
	addl	$16, %esp
	testl	%eax, %eax
	je	.L22
	subl	$12, %esp
	pushl	$.LC0
	call	puts
	addl	$16, %esp
	movl	$-1, %eax
	jmp	.L26
.L22:
	movl	$actHB_handler, -176(%ebp)
	movl	$134217728, -44(%ebp)
	subl	$4, %esp
	pushl	$0
	leal	-176(%ebp), %eax
	pushl	%eax
	pushl	$14
	call	sigaction
	addl	$16, %esp
	subl	$12, %esp
	pushl	$.LC3
	call	puts
	addl	$16, %esp
	subl	$8, %esp
	pushl	$0
	leal	-28(%ebp), %eax
	pushl	%eax
	call	gettimeofday
	addl	$16, %esp
	movl	heartBeat, %edx
	movl	heartBeat, %eax
	subl	$8, %esp
	pushl	%edx
	pushl	%eax
	call	ualarm
	addl	$16, %esp
.L25:
	movl	gDisp, %eax
	cmpl	$14999, %eax
	jbe	.L23
	movl	gStep, %eax
	cmpl	$99999, %eax
	jbe	.L23
	subl	$8, %esp
	pushl	$0
	leal	-36(%ebp), %eax
	pushl	%eax
	call	gettimeofday
	addl	$16, %esp
	movl	-36(%ebp), %eax
	movl	%eax, -188(%ebp)
	fildl	-188(%ebp)
	movl	-28(%ebp), %eax
	movl	%eax, -188(%ebp)
	fildl	-188(%ebp)
	fsubrp	%st, %st(1)
	fldl	.LC4
	fmulp	%st, %st(1)
	movl	-32(%ebp), %eax
	movl	%eax, -188(%ebp)
	fildl	-188(%ebp)
	movl	-24(%ebp), %eax
	movl	%eax, -188(%ebp)
	fildl	-188(%ebp)
	fsubrp	%st, %st(1)
	fldl	.LC4
	fdivrp	%st, %st(1)
	faddp	%st, %st(1)
	fldl	.LC4
	fdivrp	%st, %st(1)
	subl	$4, %esp
	leal	-8(%esp), %esp
	fstpl	(%esp)
	pushl	$.LC5
	call	printf
	addl	$16, %esp
	nop
	nop
	movl	$0, %eax
	jmp	.L26
.L23:
	subl	$12, %esp
	pushl	$5
	call	usleep
	addl	$16, %esp
	jmp	.L25
.L26:
	movl	-4(%ebp), %ecx
	.cfi_def_cfa 1, 0
	leave
	.cfi_restore 5
	leal	-4(%ecx), %esp
	.cfi_def_cfa 4, 4
	ret
	.cfi_endproc
.LFE5:
	.size	main, .-main
	.local	cnt.3982
	.comm	cnt.3982,4,4
	.local	cnt.3990
	.comm	cnt.3990,4,4
	.section	.rodata
	.align 8
.LC4:
	.long	0
	.long	1083129856
	.ident	"GCC: (GNU) 5.2.0"
	.section	.note.GNU-stack,"",@progbits
