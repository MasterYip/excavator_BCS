#ifndef __FILTER_H
#define __FILTER_H	 
#ifdef __cplusplus
extern "C" {
#endif
#include "stdio.h"

typedef struct
{
	float xv[3];
	float yv[3];
	float input;
	float output;
}Bw30HzLPFTypeDef;

typedef struct
{
	float xv[4];
	float yv[4];
	float input;
	float output;
}Bw50HzLPFTypeDef;

typedef struct
{
	Bw50HzLPFTypeDef GyroxLPF;
	Bw50HzLPFTypeDef GyroyLPF;
	Bw50HzLPFTypeDef GyrozLPF;
	Bw30HzLPFTypeDef AccxLPF;
	Bw30HzLPFTypeDef AccyLPF;
	Bw30HzLPFTypeDef AcczLPF;
}Filter6axisTypeDef;



extern Filter6axisTypeDef Filters_1;
extern Filter6axisTypeDef Filters_2;
extern Filter6axisTypeDef Filters_3;
extern Filter6axisTypeDef Filters_4;

void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF);
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF);
void LPFUpdate6axis_1(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az);
void LPFUpdate6axis_2(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az);
void LPFUpdate6axis_3(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az);
void LPFUpdate6axis_4(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az);

#ifdef __cplusplus
}
#endif
#endif
