#include "real_kheperaiv_ultrasound_sensor.h"

/****************************************/
/****************************************/

CRealKheperaIVUltrasoundSensor::CRealKheperaIVUltrasoundSensor(knet_dev_t* pt_dspic) :
   CRealKheperaIVDevice(pt_dspic) {
   kh4_activate_us(0x1F, GetDSPic());
}
   
/****************************************/
/****************************************/

CRealKheperaIVUltrasoundSensor::~CRealKheperaIVUltrasoundSensor() {
   kh4_activate_us(0, GetDSPic());
}

/****************************************/
/****************************************/

#define SETREADING(ARGOSIDX, KH4IDX)                                    \
   m_tReadings[ARGOSIDX].Value = (GetBuffer()[KH4IDX*2] | GetBuffer()[KH4IDX*2+1] << 8);

void CRealKheperaIVUltrasoundSensor::Do(Real f_elapsed_time) {
   kh4_measure_us(GetBuffer(), GetDSPic());
   SETREADING(0, 2);
   SETREADING(1, 1);
   SETREADING(2, 0);
   SETREADING(3, 4);
   SETREADING(4, 3);
}

/****************************************/
/****************************************/
