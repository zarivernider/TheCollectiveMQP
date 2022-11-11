
#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/src/khepera.h>
#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/template/cgripperI2C.h>
#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/template/cgripperI2C.c>

static int quitReq = 0;



/*! handle to the various Gripper devices (knet socket, i2c mode)
 */
static knet_dev_t * Turret;
static knet_dev_t * Gripper;

int initGripper( void )
{
  cgripper_init();
  /* open various socket and store the handle in their respective pointers */
  Turret = knet_open( "Cgripper:Turret" , KNET_BUS_I2C , 0 , NULL );
  Gripper  = knet_open( "Cgripper:Gripper" , KNET_BUS_I2C , 0 , NULL );

  if(Turret!=0)
  {
    if(Gripper!=0)
    {
      return 0;
    }
    else
      return -1;
  }

  return -2;

}

int main( int arc, char *argv[])
{
  char i;

	char buf[64];

  printf("Khepera3 Gripper test program (C) K-Team S.A\r\n");
  
  unsigned short message = 100;

  if(!initGripper())
  {
    printf("Init oke...\r\n");
    printf("Testing please work");

    while (!quitReq)
    {

      cgripper_Turret_Set_Max_Speed(Gripper,message);
      printf("\n> ");
      printf("sent");

    }

    printf("Exiting...\r\n");
	}
	else
	  printf("Fatal error, unable to initialize since something chandler"
			  "did is wrong\r\n");

}
