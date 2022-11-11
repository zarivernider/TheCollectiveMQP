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

#include "khepera/khepera.h"
#include "cgripperI2C.h"

int cgripper_init( void ){
	int rc;

	 /* First of all this function initializes the khepera library */

	 if((rc = kb_init( 0 , NULL )) < 0 )
	 {
	 	/* Unable to initialize the khepera library */
		KB_ERROR("kb_kh4_init",KB_ERROR_KH4KBINIT);
		return KH4_ERROR_KBINIT;
	 }
}

unsigned short cgripper_Turret_Get_Position( knet_dev_t * dev ){
	unsigned char Position;
	knet_read8( dev , ARM_VERSION , &Position );

	return Position;
}
unsigned short cgripper_Turret_Get_Speed( knet_dev_t * dev ){
	unsigned char Position;
	knet_read8( dev , ARM_VERSION , &Position );

	return Position;
}
unsigned short cgripper_Turret_Get_Order( knet_dev_t * dev ){
	unsigned char Order;
	knet_read8( dev , ARM_VERSION , &Order );

	return Order;
}
unsigned short cgripper_Turret_Get_Max_Speed( knet_dev_t * dev ){
	unsigned char Max_Speed;
	knet_read8( dev , ARM_VERSION , &Max_Speed );

	return Max_Speed;
}
unsigned short cgripper_Turret_Get_Max_Tolerance( knet_dev_t * dev ){
	unsigned char Max_Tolerance;
	knet_read8( dev , ARM_VERSION , &Max_Tolerance );

	return Max_Tolerance;
}
void cgripper_Turret_Set_Order( knet_dev_t * dev, unsigned short Order){

}
void cgripper_Turret_Set_Max_Speed( knet_dev_t * dev, unsigned short Max_Speed){

}
void cgripper_Turret_Set_Max_Tolerance( knet_dev_t * dev, unsigned short Max_Tolerance){

}
/* Gripper Functions */
unsigned short cgripper_Gripper_Get_Position( knet_dev_t * dev ){

}
//extern unsigned short cgripper_Gripper_Get_Speed( knet_dev_t * dev );
unsigned short cgripper_Gripper_Get_Order( knet_dev_t * dev ){

}
void cgripper_Gripper_Set_Position( knet_dev_t * dev ){

}
//extern void cgripper_Gripper_Set_Speed( knet_dev_t * dev );
void cgripper_Gripper_Set_Order( knet_dev_t * dev ){

}
/* Force Sensor Functions */
unsigned short cgripper_ForceSensor_Get_Force( knet_dev_t * dev ){

}
//extern unsigned short cgripper_ForceSensor2_Get_Force( knet_dev_t * dev );
//extern unsigned short cgripper_ForceSensor3_Get_Force( knet_dev_t * dev );
/* LED Functions */
unsigned short cgripper_LEDRing_Get_Status( knet_dev_t * dev ){

}
void cgripper_LEDRing_Set_Status( knet_dev_t * dev ){

}
