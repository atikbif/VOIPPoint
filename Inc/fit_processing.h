/*
 * fit_processing.h
 *
 *  Created on: 2 θών. 2019 γ.
 *      Author: User
 */

#ifndef FIT_PROCESSING_H_
#define FIT_PROCESSING_H_

#include "arm_math.h"

/* Private defines -----------------------------------------------------------*/
#define BLOCK_SIZE                      32
#define NUM_TAPS                        29
#define NUM_FIR_TAPS_Q15                56
//#define BLOCKSIZE                       32

void FIR_Init(void);
void FIR_PROCESSING_F32Process(float32_t *inp, float32_t *out);
void FIR_PROCESSING_Q31Process(void);
void FIR_PROCESSING_Q15Process(int LP_or_HP);

#endif /* FIT_PROCESSING_H_ */
