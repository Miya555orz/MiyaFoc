#include "LowPassFilter.h"
 
float Low_Pass_Filter(float Raw_Val,float Filter_Val,float Alpha)
{
	return Alpha*Raw_Val+(1.0-Alpha)*Filter_Val;
}



