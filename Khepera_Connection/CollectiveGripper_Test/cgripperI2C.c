/*!
 * \file   cgripperI2C.c The Collective Khepera4 Gripper Module
 *
 * \brief
 *         This module is layer for communication with the collective's khepera 4 Gripper module.
 *
 * \author   Chandler Garcia & Yasmine Aoua
 *
 * \note     We are using the kb_gripper code for reference
 * \bug      none discovered.
 * \todo     Send via I2C, initialize the gripper module, finish out MQP.
 */

#include "../libkhepera-2.1/src/khepera.h"
#include "../libkhepera-2.1/src/utils/i2ctest.c"
// #include <../template/cgripperI2C.h> doesn't exists???
#include "cgripperI2C.h"

i2c_t i2c;
// TODO : Check if this devpath even works (seems to worlk - Yasmine)
char *devpath = NULL;// might default to correct devpath, but unsure
int status;
int addr = 0x24; // CONFIRMED CORRECT

int cgripper_init( void ){
	/* Initializes the connection to the gripper, expects a response
	 * TODO : Wait for a response for a specific amount of time*/

	fprintf(stderr, "Opening i2c device %s\n", devpath);

	  if (i2c_open(&i2c, devpath) < 0) {
	    perror("open");
	    exit(1);
	    return -1;
	  }
	  return 0;
}

int close_gripper(void){
	i2c_close(&i2c);
}

unsigned short cgripper_Turret_Get_Position(){
	unsigned short Position;
	status = i2c_read16(&i2c,addr,0x1,&Position);
	unsigned short DegPosition = Position*360/44690;
	return Position;
}
unsigned short cgripper_Turret_Get_Speed(){
	unsigned short Speed;
	status = i2c_read16(&i2c,addr,TURRET_SPEED,&Speed);
	return Speed;
}

unsigned short cgripper_Turret_Get_Max_Speed(){
	unsigned short Max_Speed;
//	knet_read8( dev , TURRET_MAX_SPEED , &Max_Speed );
	status = i2c_read16(&i2c,addr,TURRET_MAX_SPEED,&Max_Speed);
	return Max_Speed;
}
unsigned short cgripper_Turret_Get_Max_Tolerance(){
	unsigned short Max_Tolerance;
//	knet_read8( dev , TURRET_MAX_TOLERANCE , &Max_Tolerance );
	status = i2c_read16(&i2c,addr,TURRET_MAX_TOLERANCE,&Max_Tolerance);
	return Max_Tolerance;
}

void cgripper_Turret_Set_Max_Speed( unsigned short Max_Speed){
//	knet_write16( dev , TURRET_MAX_SPEED , Max_Speed );
	status = i2c_write16(&i2c, addr, TURRET_MAX_SPEED, Max_Speed);
}
void cgripper_Turret_Set_Max_Tolerance( unsigned short Max_Tolerance){
	status = i2c_write16(&i2c, addr, TURRET_MAX_TOLERANCE, Max_Tolerance);
}
/* Gripper Functions */
unsigned short cgripper_Gripper_Get_Position(){
	unsigned short Position;
//	knet_read8( dev , GRIPPER_POSITION , &Position );
	// i2c_read16(&i2c,addr,GRIPPER_POSITION,&Position); // for use on khepera
	i2c_read16(&i2c,addr,0x1,&Position); // for testing on pi pico
	return Position;
}

void cgripper_Gripper_Set_Position( unsigned short Position){
	// i2c_write16(&i2c,addr,GRIPPER_POSITION,Position); // for use on khepera
	i2c_write16(&i2c,addr,0x1,Position); // for testing on pi pico
}

/* Force Sensor Functions */
unsigned short cgripper_ForceSensor_Get__Parallel_Force(){
	unsigned short Force;
//	knet_read8( dev , FORCE_SENSOR_FORCE , &Force );
	i2c_read16(&i2c,addr,FORCE_SENSOR_PARALLEL,&Force);
	return Force;
}
unsigned short cgripper_ForceSensor_Get__Perpendicular_Force(){
	unsigned short Force;
//	knet_read8( dev , FORCE_SENSOR_FORCE , &Force );
	i2c_read16(&i2c,addr,FORCE_SENSOR_PERPENDICULAR,&Force);
	return Force;
}
//extern unsigned short cgripper_ForceSensor3_Get_Force( knet_dev_t * dev );
/* LED Functions */
unsigned short cgripper_LEDRing_Get_Config(){
	unsigned short Config;
//	knet_read8( dev , LED_STATUS , &Config );
	i2c_read16(&i2c,addr,LED_CONFIG,&Config);
	return Config;

}
void cgripper_LEDRing_Set_Status( unsigned short Config){
	i2c_write16(&i2c,addr,LED_CONFIG,Config);
}

void testLED() {
  i2c_write16(&i2c, addr, 0x16, 0x6DB6);
  sleep(1);
  i2c_write16(&i2c, addr, 0x17, 0xDB6D);
  sleep(1);
  i2c_write16(&i2c, addr, 0x18, 0xB6DB);
  sleep(1);
  i2c_write16(&i2c, addr, 0x19, 0x6DB6);
  sleep(1);
  i2c_write16(&i2c, addr, 0x1A, 0x0001);
  sleep(1);
}

void rotateTurret(){
	// i2c_write16(&i2c, addr, 0x3, 1001); // reset the max speed bc zach accidentally flashed it w a maz of 0
	// // 0x2 to decimal value of 1000 (SETTING SPEED)
	// i2c_write16(&i2c, addr, 0x2, 1000);
	// sleep(1);
	// 0x8 to 2
	i2c_write16(&i2c, addr, 0x8, 0x2);
	sleep(1);
	// stop turning
	// i2c_write16(&i2c, addr, 0x8, 0);
	// sleep(1);
}
