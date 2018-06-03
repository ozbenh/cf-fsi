	.data
	.align 8
	.globl cf_code_start
cf_code_start:
	.incbin "cf-code/cf-fsi-romulus.bin"
	.align 8
	.globl cf_code_end
cf_code_end:
	.previous
