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

#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/src/knet.h>

/*--------------------------------------------------------------------
 *!  KoreMotor Order Mask (See knet_set_order)
 *!  What this is for? I don't know - Chandler
 */
#define CGRIPPER_ORDER_MASK ( KNET_ORDER_LITTLE | \
		KNET_ORDER_REP_ADR | KNET_ORDER_LSB_FIRST)


/*--------------------------------------------------------------------
 *	Turret Register Definitions
 */
#define TURRET_POSITION 		0x31
#define TURRET_SPEED 			0x32
#define TURRET_ORDER 			0x33 // Register to send orders over
#define TURRET_MAX_SPEED		0x34
#define TURRET_MAX_TOLERANCE	0x35 // + or - in degrees of turret angle

/*--------------------------------------------------------------------
 *	Gripper Register Definitions
 */
#define CGRIPPER_POSITION		0x36
//#define GRIPPER_SPEED			0x37 // We probably wouldn't need this
#define CGRIPPER_ORDER			0x38

/*--------------------------------------------------------------------
 *	Force Sensor Register Definitions
 *	Multiple are included as I have no idea if we will use them ever
 */
#define FORCE_SENSOR_FORCE		0x39
//#define FORCE_SENSOR2_FORCE		0x3A
//#define FORCE_SENSOR3_FORCE		0x3B

/*--------------------------------------------------------------------
 *	LED Register Definitions
 *	Unsure if we would ever want to know what we are showing on LED,
 *	or just send orders
 */
#define LED_STATUS				0x3C
#define LED_ORDER				0x3D

/*--------------------------------------------------------------------
 *	Function Definitions
 *	Every return is a short since we will probably have to do
 *	conversions ourselves.
 */
extern int cgripper_init(void);
/* Turret Functions */
extern unsigned short cgripper_Turret_Get_Position( knet_dev_t * dev );
extern unsigned short cgripper_Turret_Get_Speed( knet_dev_t * dev );
extern unsigned short cgripper_Turret_Get_Order( knet_dev_t * dev );
extern unsigned short cgripper_Turret_Get_Max_Speed( knet_dev_t * dev );
extern unsigned short cgripper_Turret_Get_Max_Tolerance( knet_dev_t * dev );
extern void cgripper_Turret_Set_Order( knet_dev_t * dev, unsigned short Order);
extern void cgripper_Turret_Set_Max_Speed( knet_dev_t * dev, unsigned short Max_Speed);
extern void cgripper_Turret_Set_Max_Tolerance( knet_dev_t * dev, unsigned short Max_Tolerance);
/* Gripper Functions */
extern unsigned short cgripper_Gripper_Get_Position( knet_dev_t * dev );
//extern unsigned short cgripper_Gripper_Get_Speed( knet_dev_t * dev );
extern unsigned short cgripper_Gripper_Get_Order( knet_dev_t * dev );
extern void cgripper_Gripper_Set_Position( knet_dev_t * dev );
//extern void cgripper_Gripper_Set_Speed( knet_dev_t * dev );
extern void cgripper_Gripper_Set_Order( knet_dev_t * dev );
/* Force Sensor Functions */
extern unsigned short cgripper_ForceSensor_Get_Force( knet_dev_t * dev );
//extern unsigned short cgripper_ForceSensor2_Get_Force( knet_dev_t * dev );
//extern unsigned short cgripper_ForceSensor3_Get_Force( knet_dev_t * dev );
/* LED Functions */
extern unsigned short cgripper_LEDRing_Get_Status( knet_dev_t * dev );
extern void cgripper_LEDRing_Set_Status( knet_dev_t * dev );


#endif /*	cgripperI2C	*/
