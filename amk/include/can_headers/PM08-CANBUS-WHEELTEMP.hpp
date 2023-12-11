//Generated on Wed Jun 15 10:27:25 2022
#ifndef WHEELTEMP
#define WHEELTEMP

#include <cstdint>

namespace PUTM_CAN {

struct __attribute__ ((packed)) WheelTemp_main{
	uint8_t wheelTemp[8]; 
};


const uint16_t WHEELTEMP_MAIN_CAN_ID = 0x69;
const uint8_t WHEELTEMP_MAIN_CAN_DLC = sizeof(WheelTemp_main);
const uint8_t WHEELTEMP_MAIN_FREQUENCY = 1;

} //namespace can

#endif

