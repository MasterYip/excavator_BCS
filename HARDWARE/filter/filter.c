#include "filter.h"

Filter6axisTypeDef Filters_1;
Filter6axisTypeDef Filters_2;
Filter6axisTypeDef Filters_3;
Filter6axisTypeDef Filters_4;

#define GAINBtw50hz   (3.450423889e+02f)
void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF)
{ 
	//¶þ½×²î·Ö
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
// Low pass filtering_1
void LPFUpdate6axis_1(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az)
{
	Filters_1.GyroxLPF.input = gyroGx;
	Filters_1.GyroyLPF.input = gyroGy;
	Filters_1.GyrozLPF.input = gyroGz;
	Butterworth50HzLPF(&Filters_1.GyroxLPF);
	Butterworth50HzLPF(&Filters_1.GyroyLPF);
	Butterworth50HzLPF(&Filters_1.GyrozLPF);
	
	Filters_1.AccxLPF.input = ax;
	Filters_1.AccyLPF.input = ay;
	Filters_1.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters_1.AccxLPF);
	Butterworth30HzLPF(&Filters_1.AccyLPF);
	Butterworth30HzLPF(&Filters_1.AcczLPF);
}


/////////////////////////////////////////////////
// Low pass filtering_2
void LPFUpdate6axis_2(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az)
{
	Filters_2.GyroxLPF.input = gyroGx;
	Filters_2.GyroyLPF.input = gyroGy;
	Filters_2.GyrozLPF.input = gyroGz;
	Butterworth50HzLPF(&Filters_2.GyroxLPF);
	Butterworth50HzLPF(&Filters_2.GyroyLPF);
	Butterworth50HzLPF(&Filters_2.GyrozLPF);
	
	Filters_2.AccxLPF.input = ax;
	Filters_2.AccyLPF.input = ay;
	Filters_2.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters_2.AccxLPF);
	Butterworth30HzLPF(&Filters_2.AccyLPF);
	Butterworth30HzLPF(&Filters_2.AcczLPF);
}


/////////////////////////////////////////////////
// Low pass filtering_3
void LPFUpdate6axis_3(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az)
{
	Filters_3.GyroxLPF.input = gyroGx;
	Filters_3.GyroyLPF.input = gyroGy;
	Filters_3.GyrozLPF.input = gyroGz;
	Butterworth50HzLPF(&Filters_3.GyroxLPF);
	Butterworth50HzLPF(&Filters_3.GyroyLPF);
	Butterworth50HzLPF(&Filters_3.GyrozLPF);
	
	Filters_3.AccxLPF.input = ax;
	Filters_3.AccyLPF.input = ay;
	Filters_3.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters_3.AccxLPF);
	Butterworth30HzLPF(&Filters_3.AccyLPF);
	Butterworth30HzLPF(&Filters_3.AcczLPF);
}


/////////////////////////////////////////////////
// Low pass filtering_4
void LPFUpdate6axis_4(float gyroGx,float gyroGy,float gyroGz,short ax,short ay,short az)
{
	Filters_4.GyroxLPF.input = gyroGx;
	Filters_4.GyroyLPF.input = gyroGy;
	Filters_4.GyrozLPF.input = gyroGz;
	Butterworth50HzLPF(&Filters_4.GyroxLPF);
	Butterworth50HzLPF(&Filters_4.GyroyLPF);
	Butterworth50HzLPF(&Filters_4.GyrozLPF);
	
	Filters_4.AccxLPF.input = ax;
	Filters_4.AccyLPF.input = ay;
	Filters_4.AcczLPF.input = az;
	Butterworth30HzLPF(&Filters_4.AccxLPF);
	Butterworth30HzLPF(&Filters_4.AccyLPF);
	Butterworth30HzLPF(&Filters_4.AcczLPF);
}

