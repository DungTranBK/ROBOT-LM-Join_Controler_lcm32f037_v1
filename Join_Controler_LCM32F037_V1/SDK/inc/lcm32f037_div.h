/*****************************************************
**  @brief div moudle driver
**	@author zhang
**	@data 2020-10-13
**	@function  A/B = Q......Rem
*******************************************************/

#ifndef LCM32F037_DIV_H
#define LCM32F037_DIV_H
#include "lcm32f037.h"
#include <stdint.h>
#include <core_cm0.h>
typedef union
{
	struct
	{
		uint32_t LOW32;
		uint32_t HIGH32;
	} D;
	uint64_t Dividend64;
} D_Typedef;

typedef struct
{
	D_Typedef result64;	  //商
	uint32_t remainder64; //余数
} DIV_Result;

long long Fun_32_DIV_32(int dividend, int divisor, DIV_Result *result);
long long Fun_64_DIV_32(long long dividend, int divisor, DIV_Result *result);

#endif
