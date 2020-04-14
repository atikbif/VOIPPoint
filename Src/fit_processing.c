/*
 * fit_processing.c
 *
 *  Created on: 2 июн. 2019 г.
 *      Author: User
 */

#include "fit_processing.h"

// FIR фильтры (коэффициенты конкретных фильтров рассчитывались с помощью внешней программы)

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = 160*6/BLOCK_SIZE;
uint32_t numBlocks2 = 160*6/BLOCK_SIZE;


float32_t aFIR_F32_Coeffs[NUM_TAPS] = {
-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
-0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
+0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
+0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

float32_t aFIR_F32_Coefffs2[NUM_TAPS2] = {
		-0.165522818886457840,
		-0.019803814563708278,
		-0.020740235915017568,
		-0.022015177140735472,
		-0.022747987016282643,
		-0.023608371388582888,
		-0.024512331042539640,
		-0.025214610770968891,
		-0.025857999369656881,
		-0.026468280126258074,
		-0.026915865589341296,
		-0.027253354765038221,
		-0.027534693187185998,
		-0.027680274814202991,
		+0.972292581524307420,
		-0.027680274814202991,
		-0.027534693187185998,
		-0.027253354765038221,
		-0.026915665589384129,
		-0.026468280126258074,
		-0.025857999869656881,
		-0.025214610770968891,
		-0.024512331042539640,
		-0.023608371338582888,
		-0.022747987016282643,
		-0.022015177140735422,
		-0.020740235915017568,
		-0.019803814563703278,
		-0.165522818886457840
};
/* ----------------------------------------------------------------------
** low pass at 1KHz with 40dB at 1.5KHz for SR=16KHz.
** ------------------------------------------------------------------- */

q15_t aFIR_Q15_Coeffs_LP[NUM_FIR_TAPS_Q15] = {
-217	,   40	,  120,  237,  366,  475,  527,  490,  346,
100		, -217	, -548, -818, -947, -864, -522,   86,  922,
1904	, 2918	, 3835, 4529, 4903, 4903, 4529, 3835, 2918,
1904	,  922	,   86, -522, -864, -947, -818, -548, -217,
100		,  346	,  490,  527,  475,  366,  237,  120,   40,
-217	,    0	,    0,    0,    0,    0,    0,    0,    0,
0,    0};
/* ----------------------------------------------------------------------
** high pass at 1.5KHz with 40dB at 1KHz for SR=16KHz
** ------------------------------------------------------------------- */

q15_t aFIR_Q15_Coeffs_HP[NUM_FIR_TAPS_Q15] = {
-654	,  483	,  393,  321,  222,   76, -108, -299, -447,
-501	, -422	, -200,  136,  520,  855, 1032,  953,  558,
-160	,-1148	,-2290,-3432,-4406,-5060,27477,-5060,-4406,
-3432	,-2290	,-1148, -160,  558,  953, 1032,  855,  520,
136		, -200	, -422, -501, -447, -299, -108,   76,  222,
321		,  393	,  483, -654,    0,    0,    0,    0,    0,
0			,    0	,};

float32_t 	firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
float32_t 	firStateF32_2[BLOCK_SIZE + NUM_TAPS2 - 1];

q31_t 			firStateQ31[BLOCK_SIZE + NUM_TAPS - 1];
q15_t 			firStateQ15[NUM_FIR_TAPS_Q15 + BLOCK_SIZE];
q31_t 		aFIR_Q31_Coeffs[NUM_TAPS];

//float32_t 	aFIR_F32_Output[160];
arm_fir_instance_f32 FIR_F32_Struct;
arm_fir_instance_f32 FIR_F32_Struct2;

void FIR_Init(void) {
	arm_fir_init_f32(&FIR_F32_Struct, NUM_TAPS, (float32_t *)&aFIR_F32_Coeffs[0], &firStateF32[0], blockSize);
	arm_fir_init_f32(&FIR_F32_Struct2, NUM_TAPS2, (float32_t *)&aFIR_F32_Coefffs2[0], &firStateF32_2[0], blockSize);
}

void FIR_PROCESSING_F32Process(float32_t *inp, float32_t *out) {

	uint32_t counter_FIR_f32_p;

	for (counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks; counter_FIR_f32_p++)
	{
		arm_fir_f32(&FIR_F32_Struct, inp + (counter_FIR_f32_p * blockSize), out + (counter_FIR_f32_p * blockSize), blockSize);
	}
}

void FIR_PROCESSING_F32Process2(float32_t *inp, float32_t *out) {

	uint32_t counter_FIR_f32_p;

	for (counter_FIR_f32_p = 0; counter_FIR_f32_p < numBlocks2; counter_FIR_f32_p++)
	{
		arm_fir_f32(&FIR_F32_Struct2, inp + (counter_FIR_f32_p * blockSize), out + (counter_FIR_f32_p * blockSize), blockSize);
	}
}


