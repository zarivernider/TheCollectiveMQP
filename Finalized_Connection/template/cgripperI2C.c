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
 * \todo     Save all the information that we send to the gripper turret in here on variables maybes?
 */

#include "../src/khepera.h"
#include "i2c_Collective.c"
#include "cgripperI2C.h"

i2c_t i2c;
// TODO : Check if this devpath even works (seems to worlk - Yasmine)
char *devpath = NULL;// might default to correct devpath, but unsure
int status;
int addr = 0x24; // CONFIRMED CORRECT
float m_PI = 3.14159265359;

int cgripper_init( void ){
	/* 
	 * Initializes the connection to the gripper, expects a response
	 * TODO : Wait for a response for a specific amount of time maybe?
	 */

	fprintf(stderr, "Opening i2c device %s\n", devpath);

	  if (i2c_open(&i2c, devpath) < 0) {
	    perror("open");
	    exit(1);
	    return -1;
	  }
	  return 0;
}

int close_gripper(void){
	/*
	 * Close the gripper module
	*/

	// stop turning
	i2c_write16(&i2c, addr, 0x8, 0);
	sleep(1);

	// turn off LEDs
	i2c_write16(&i2c, addr, 0x16, 0);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x1A, 0x001);
	sleep(1);

	// Remove the i2c connection
	i2c_close(&i2c);
}

void cgripper_Modularity_Test(){
	i2c_write16(&i2c, addr, CONTINUITY_TEST, 0x01);
}

/**********************************************/
/**************Gripper Functions***************/
/**********************************************/


unsigned short cgripper_Gripper_Get_Position(){
	/*
	 * Unsure if this will ever be used, like why the hell do you want to know
	 * where the gripper is? Did you NOT keep track of it yourself? Idiot
	*/
	unsigned short Position;
	i2c_read16(&i2c,addr,CGRIPPER_POSITION,&Position);
	return Position;
	return Position;
}

void cgripper_Open_Gripper(){
	i2c_write16(&i2c, addr, CGRIPPER_POSITION, 0x01);
}

void cgripper_Close_Gripper(){
	i2c_write16(&i2c, addr, CGRIPPER_POSITION, 0x00);
}

void cgripper_Gripper_Set_Position( unsigned short Position){
	/* 
	 * The Position is 0 closed, 1 is open, anything else is fucked up man, 
	 * why are you using this command
	*/
	i2c_write16(&i2c,addr,CGRIPPER_POSITION,Position); // for testing on pi pico
}

/**********************************************/
/***************Turret Functions***************/
/**********************************************/

/* Position Information */
unsigned short cgripper_Turret_Get_Position(){
	/*
	 * Turret Position in Degrees
	*/
	unsigned short Position;
	status = i2c_read16(&i2c,addr,TURRET_GET_POSITION,&Position);
	unsigned short DegPosition = Position*360/44690;
	return DegPosition;
}
unsigned short cgripper_Turret_Get_Raw_Position(){
	/*
	 * Turret Position RAW in PWM Pulse width waveform
	*/
	unsigned short Position;
	status = i2c_read16(&i2c,addr,RAW_TURRET_POSITION,&Position);
	return Position;
}

void cgripper_Turret_Set_Position(float pos){
	/*
	 * Takes in a number 0 to 2pi and converts that into the robot's position between 0 to 44690
	*/
	unsigned short posToEncoder = (unsigned short) pos / M_PI / 2 * 44690;
	printf("Should be sending position %hu\n", posToEncoder);
	i2c_write16(&i2c, addr, TURRET_SET_POSITION, posToEncoder);
}

/* Speed Information (Sets and Gets) */
unsigned short cgripper_Turret_Get_Speed(){
	// TODO : Update this to send the speed which we sent it to go at last time
	unsigned short Speed;
	status = i2c_read16(&i2c,addr,TURRET_SPEED,&Speed);
	return Speed;
}

void cgripper_Turret_Set_Speed(short speed){
	i2c_write16(&i2c, addr, TURRET_SPEED, speed);
}

unsigned short cgripper_Turret_Get_Max_Speed(){
	unsigned short Max_Speed;
//	knet_read8( dev , TURRET_MAX_SPEED , &Max_Speed );
	status = i2c_read16(&i2c,addr,TURRET_MAX_SPEED,&Max_Speed);
	return Max_Speed;
}

void cgripper_Turret_Set_Max_Speed( unsigned short Max_Speed){
//	knet_write16( dev , TURRET_MAX_SPEED , Max_Speed );
	status = i2c_write16(&i2c, addr, TURRET_MAX_SPEED, Max_Speed);
}

/* Degree Error Information */
unsigned short cgripper_Turret_Get_Max_Tolerance(){
	unsigned short Max_Tolerance;
//	knet_read8( dev , TURRET_MAX_TOLERANCE , &Max_Tolerance );
	status = i2c_read16(&i2c,addr,TURRET_MAX_TOLERANCE,&Max_Tolerance);
	return Max_Tolerance;
}

void cgripper_Turret_Set_Max_Tolerance( unsigned short Max_Tolerance){
	status = i2c_write16(&i2c, addr, TURRET_MAX_TOLERANCE, Max_Tolerance);
}

/* Reset Turret Encoder 0 Information */
void cgripper_Turret_Set_Autotrim(unsigned short Autotrim){
	/*
	 * Setting this to 1, we will update the encoder's recognized 0 degree for the turret
	*/
	i2c_write16(&i2c, addr, TURRET_AUTOTRIM, Autotrim);
}

void cgripper_Turret_Set_Trim(unsigned short Trim){
	i2c_write16(&i2c, addr, TURRET_TRIM, Trim);
}

/* Set the Turret's mode, you will most likely always be using Position Mode */
void cgripper_Turret_Disable(){
	i2c_write16(&i2c, addr, 0x8, 0x0);
}

void cgripper_Turret_Position_Mode(){
	i2c_write16(&i2c, addr, 0x8, 0x1);
}

void cgripper_Turret_Speed_Mode(){
	i2c_write16(&i2c, addr, 0x8, 0x2);
}

void cgripper_Turret_Locked(){
	i2c_write16(&i2c, addr, 0x8, 0x3);
}

void cgripper_Turret_Push(){
	i2c_write16(&i2c, addr, 0x8, 0x4);
}

void cgripper_Turret_Set_Proportional(unsigned short Proportional){
	i2c_write16(&i2c, addr, TURRET_PROPORTIONAL_CONSTANT, Proportional);
}

void cgripper_Turret_Set_Integral(unsigned short Integral){
	i2c_write16(&i2c, addr, TURRET_INTEGRAL_CONSTANT, Integral);
}

void cgripper_Turret_Set_Derivative(unsigned short Derivative){
	i2c_write16(&i2c, addr, TURRET_DERIVATIVE_CONSTANT, Derivative);
}

void cgripper_Turret_Force_Tolerance(unsigned short Proportional){
	i2c_write16(&i2c,addr, TURRET_FORCE_PROPORTIONAL, Proportional);
}

/**********************************************/
/*************Force Sensor Functions***********/
/**********************************************/

unsigned short cgripper_ForceSensor_Get_Parallel_Force(){
	unsigned short Force;
//	knet_read8( dev , FORCE_SENSOR_FORCE , &Force );
	i2c_read16(&i2c,addr,FORCE_SENSOR_PARALLEL,&Force);
	return Force;
}
unsigned short cgripper_ForceSensor_Get_Perpendicular_Force(){
	unsigned short Force;
//	knet_read8( dev , FORCE_SENSOR_FORCE , &Force );
	i2c_read16(&i2c,addr,FORCE_SENSOR_PERPENDICULAR,&Force);
	return Force;
}

/**********************************************/
/******************LED Functions***************/
/**********************************************/
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
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0xDB6D);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0xB6DB);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0x6DB6);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
	sleep(1);
	int i=0;i2c_write16(&i2c, addr, 0xD, i);
	for(i=0; i<32; i++){
		i2c_write16(&i2c, addr, 0xD, i);
		i2c_write16(&i2c, addr, 0x1A, 0x0001);
		usleep(500000);
	}
}

void setAllRed() {
	i2c_write16(&i2c, addr, 0x16, 0x5555);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0x5555);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0x5555);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0x5555);
	// sleep(1);
	i2c_write16(&i2c, addr, 0xD, 2);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
}

void setGrippedRed() {
	i2c_write16(&i2c, addr, 0x16, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0x556A);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0xD, 2);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
}

void setAllBlue() {
	i2c_write16(&i2c, addr, 0x16, 0xFFFF);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0xFFFF);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0xFFFF);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0xFFFF);
	// sleep(1);
	i2c_write16(&i2c, addr, 0xD, 2);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
}

void setAllGreen() {
	i2c_write16(&i2c, addr, 0x16, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0xAAAA);
	// sleep(1);
	i2c_write16(&i2c, addr, 0xD, 2);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
}

void setAllOff() {
	i2c_write16(&i2c, addr, 0x16, 0x0000);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x17, 0x0000);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x18, 0x0000);
	// sleep(1);
	i2c_write16(&i2c, addr, 0x19, 0x0000);
	// sleep(1);
	i2c_write16(&i2c, addr, 0xD, 0);
	i2c_write16(&i2c, addr, 0x1A, 0x0001);
}

void setKi(int Ki){
	i2c_write16(&i2c, addr, 0x6, Ki);
}

void setKp(int Kp){
	i2c_write16(&i2c, addr, 0x5, Kp);
}

void cgripper_Set_EEPROM(){
	i2c_write16(&i2c, addr, EEPROM_REGISTER, 1);
}

void set_turret_zero(){
	i2c_write16(&i2c, addr, TURRET_TRIM, 1);
}