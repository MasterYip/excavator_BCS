#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif


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



extern Filter6axisTypeDef Filters[6];


void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF);
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF);

void LPFUpdate6axis(float AngularVel[3],float Acceleration[3], Filter6axisTypeDef *filter);


#ifdef __cplusplus
}
#endif
#endif
