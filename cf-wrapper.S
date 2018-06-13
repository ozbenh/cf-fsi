#define _STR(f) #f
#define STR(f) _STR(f)
	.data
	.align 8
	.globl cf_code_start
cf_code_start:
	.incbin STR(CF_FILE)
	.align 8
	.globl cf_code_end
cf_code_end:
	.previous
