/*!
 * \file   cgripperI2C.c The Collective Khepera4 Gripper Module
 *
 * \brief
 *         This module is layer for communication with the collective's khepera 4 Gripper module.
 *
 * \author   Chandler Garcia & Yasmine Lastname
 *
 * \note     We are using the kb_gripper code for reference
 * \bug      none discovered.
 * \todo     Send via I2C, initialize the gripper module, finish out MQP.
 */

#include <../src/khepera.h>
#include <../template/cgripperI2C.h>

i2c_t i2c;
// TODO : Check if this devpath even works
char *devpath = NULL;// might default to correct devpath, but unsure
int status;
int addr = 0;

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
	unsigned char Position;
	status = i2c_read16(&i2c,addr,TURRET_POSITION,&Position);
	return Position;
}
unsigned short cgripper_Turret_Get_Speed(){
	unsigned char Speed;
	status = i2c_read16(&i2c,addr,TURRET_SPEED,&Speed);
	return Speed;
}

unsigned short cgripper_Turret_Get_Max_Speed(){
	unsigned char Max_Speed;
//	knet_read8( dev , TURRET_MAX_SPEED , &Max_Speed );
	status = i2c_read16(&i2c,addr,TURRET_MAX_SPEED,&Max_Speed);
	return Max_Speed;
}
unsigned short cgripper_Turret_Get_Max_Tolerance(){
	unsigned char Max_Tolerance;
//	knet_read8( dev , TURRET_MAX_TOLERANCE , &Max_Tolerance );
	status = i2c_read16(&i2c,addr,TURRET_MAX_TOLERANCE,&Max_Tolerance);
	return Max_Tolerance;
}

void cgripper_Turret_Set_Max_Speed(unsigned short Max_Speed){
//	knet_write16( dev , TURRET_MAX_SPEED , Max_Speed );
	status = i2c_write16(&i2c, addr, TURRET_MAX_SPEED, Max_Speed);
}
void cgripper_Turret_Set_Max_Tolerance(unsigned short Max_Tolerance){
	status = i2c_write16(&i2c, addr, TURRET_MAX_TOLERANCE, Max_Tolerance);
}
/* Gripper Functions */
unsigned short cgripper_Gripper_Get_Position(){
	unsigned char Position;
//	knet_read8( dev , GRIPPER_POSITION , &Position );
	i2c_read16(&i2c,addr,GRIPPER_POSITION,&Position);
	return Position;
}

void cgripper_Gripper_Set_Position(unsigned short Position){
	i2c_write16(&i2c,addr,GRIPPER_POSITION,Position);
}

/* Force Sensor Functions */
unsigned short cgripper_ForceSensor_Get_Force(){
	unsigned char Force;
//	knet_read8( dev , FORCE_SENSOR_FORCE , &Force );
	i2c_read16(&i2c,addr,FORCE_SENSOR_FORCE,&Force);
	return Force;
}
//extern unsigned short cgripper_ForceSensor2_Get_Force( knet_dev_t * dev );
//extern unsigned short cgripper_ForceSensor3_Get_Force( knet_dev_t * dev );
/* LED Functions */
unsigned short cgripper_LEDRing_Get_Config(){
	unsigned char Config;
//	knet_read8( dev , LED_STATUS , &Config );
	i2c_read16(&i2c,addr,LED_CONFIG,&Config);
	return Config;

}
void cgripper_LEDRing_Set_Status(unsigned short Config){
	i2c_write16(&i2c,addr,LED_CONFIG,Config);
}
