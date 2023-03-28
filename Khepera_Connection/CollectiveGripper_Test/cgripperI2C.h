/*--------------------------------------------------------------------
 * kb_gripper.h - KoreBot Library - Khepera3 Gripper functions
 *--------------------------------------------------------------------
 * $Author: crgarcia@wpi.edu $
 * $Date: 2022/11/10
 * $Revision: 1.0 $
 * Using kb_gripper for reference
 *-------------------------------------------------------------------*/

#ifndef __cgripperI2C__
#define __cgripperI2C__

#include "../libkhepera-2.1/src/knet.h"
// #include "../template/cgripperI2C.c" doesn't exist???
#include "../libkhepera-2.1/src/i2ccom.h"

/*--------------------------------------------------------------------
 *!  KoreMotor Order Mask (See knet_set_order)
 *!  What this is for? I don't know - Chandler
 */

/*--------------------------------------------------------------------
 *	Turret Register Definitions (most are wrong unless otherwise specified)
 */
#define TURRET_POSITION 		0x1E // THIS IS CORRECT
#define TURRET_ZERO             0x1D // CORRECT
#define TURRET_SPEED 			0x32
#define TURRET_ORDER 			0x33 // Register to send orders over
#define TURRET_MAX_SPEED		0x34
#define TURRET_MAX_TOLERANCE	0x35 // + or - in degrees of turret angle

/*--------------------------------------------------------------------
 *	Gripper Register Definitions
 */
#define GRIPPER_POSITION		0x36
//#define GRIPPER_SPEED			0x37 // We probably wouldn't need this

/*--------------------------------------------------------------------
 *	Force Sensor Register Definitions
 *	Multiple are included as I have no idea if we will use them ever
 */
#define FORCE_SENSOR_PARALLEL		0xB // THIS IS CORRECT
#define FORCE_SENSOR_PERPENDICULAR	0xC // THIS IS CORRECT
//#define FORCE_SENSOR3_FORCE		0x3B

/*--------------------------------------------------------------------
 *	LED Register Definitions
 *	Unsure if we would ever want to know what we are showing on LED,
 *	or just send orders
 */
#define LED_CONFIG				0x16

#define EEPROM                  0x1B // CORRECT

/*--------------------------------------------------------------------
 *	Function Definitions
 *	Every return is a short since we will probably have to do
 *	conversions ourselves.
 */

extern int cgripper_init(void);
int close_gripper(void);
/* Turret Functions */
extern unsigned short cgripper_Turret_Get_Position();
extern unsigned short cgripper_Turret_Get_Speed();

extern unsigned short cgripper_Turret_Get_Max_Speed();
extern unsigned short cgripper_Turret_Get_Max_Tolerance();
extern void cgripper_Turret_Set_Max_Speed( unsigned short Max_Speed);
extern void cgripper_Turret_Set_Max_Tolerance( unsigned short Max_Tolerance);
/* Gripper Functions */
extern unsigned short cgripper_Gripper_Get_Position();
//extern unsigned short cgripper_Gripper_Get_Speed( knet_dev_t * dev );
extern void cgripper_Gripper_Set_Position( unsigned short Position );
//extern void cgripper_Gripper_Set_Speed( knet_dev_t * dev );
/* Force Sensor Functions */
extern unsigned short cgripper_ForceSensor_Get_Force();
//extern unsigned short cgripper_ForceSensor2_Get_Force( knet_dev_t * dev );
//extern unsigned short cgripper_ForceSensor3_Get_Force( knet_dev_t * dev );
/* LED Functions */
extern unsigned short cgripper_LEDRing_Get_Config();
extern void cgripper_LEDRing_Set_Status( unsigned short Config );
void testLED();
void rotateTurret();
void set_turret_zero();
void set_EEPROM();
void stop();

#endif /*	cgripperI2C	*/
