#include "filter.h"

Filter6axisTypeDef Filters[6];

#define GAINBtw50hz   (3.450423889e+02f)
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF)
{ 
	//二阶差分
      pLPF->xv[0] = pLPF->xv[1]; pLPF->xv[1] = pLPF->xv[2]; pLPF->xv[2] = pLPF->xv[3]; 
        pLPF->xv[3] = pLPF->input / GAINBtw50hz;
        pLPF->yv[0] = pLPF->yv[1]; pLPF->yv[1] = pLPF->yv[2]; pLPF->yv[2] = pLPF->yv[3]; 
        pLPF->yv[3] =   (pLPF->xv[0] + pLPF->xv[3]) + 3 * (pLPF->xv[1] + pLPF->xv[2])
                     + (  0.5320753683f * pLPF->yv[0]) + ( -1.9293556691f * pLPF->yv[1])
                     + (  2.3740947437f * pLPF->yv[2]);
        
	pLPF->output = pLPF->yv[3];
}

#define GAINBtw30Hz   1.278738361e+02f
void Butterworth30HzLPF(Bw30HzLPFTypeDef* pLPF)
{
	pLPF->xv[0] = pLPF->xv[1];
	pLPF->xv[1] = pLPF->xv[2]; 
  pLPF->xv[2] = pLPF->input / GAINBtw30Hz;
  pLPF->yv[0] = pLPF->yv[1];
	pLPF->yv[1] = pLPF->yv[2]; 
  pLPF->yv[2] = (pLPF->xv[0] + pLPF->xv[2]) + 2 * pLPF->xv[1]
                     + ( -0.7660066009f * pLPF->yv[0]) + (  1.7347257688f * pLPF->yv[1]);
  pLPF->output = pLPF->yv[2];
}


/////////////////////////////////////////////////
// Low pass filtering

/**
  * @brief  六轴数据低通滤波
  * @note   
  * @param  float AngularVel[3] 输入的原始角速度
  * @param  float Acceleration[3] 输入的原始角速度
  * @retval Filter6axisTypeDef filter 更新的滤波器数据以及结果
  */
void LPFUpdate6axis(float AngularVel[3], float Acceleration[3], Filter6axisTypeDef *filter)
{
	filter->GyroxLPF.input = AngularVel[0];
	filter->GyroyLPF.input = AngularVel[1];
	filter->GyrozLPF.input = AngularVel[2];
	Butterworth50HzLPF(&filter->GyroxLPF);
	Butterworth50HzLPF(&filter->GyroyLPF);
	Butterworth50HzLPF(&filter->GyrozLPF);
	
	filter->AccxLPF.input = Acceleration[0];
	filter->AccyLPF.input = Acceleration[1];
	filter->AcczLPF.input = Acceleration[2];
	Butterworth30HzLPF(&filter->AccxLPF);
	Butterworth30HzLPF(&filter->AccyLPF);
	Butterworth30HzLPF(&filter->AcczLPF);
}

