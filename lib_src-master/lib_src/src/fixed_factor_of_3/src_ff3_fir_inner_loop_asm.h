// Copyright (c) 2016, XMOS Ltd, All rights reserved
#ifndef _SRC_FF3_FIR_INNER_LOOP_ASM_H_
#define _SRC_FF3_FIR_INNER_LOOP_ASM_H_

#define SRC_FF3_N_LOOPS_PER_ASM   12

void src_ff3_fir_inner_loop_asm(int *piData, int *piCoefs, int iData[], int count);
void src_ff3_fir_inner_loop_asm_odd(int *piData, int *piCoefs, int iData[], int count);


#endif // _SRC_FF3_FIR_INNER_LOOP_ASM_H_
