
#include <khepera/khepera.h>


static int quitReq = 0;



/*! handle to the various Gripper devices (knet socket, i2c mode)
 */
static knet_dev_t * Turret;
static knet_dev_t * Gripper;

int initGripper( void )
{

  kgripper_init();
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

  if(!initGripper())
  {
    printf("Init oke...\r\n");
    printf("Testing please work");

    while (!quitReq)
    {


      printf("\n> ");

      if ( fgets( buf , sizeof(buf) , stdin ) != NULL )
      {
				buf[strlen(buf)-1] = '\0';
				kb_parse_command( buf , cmds , NULL);
      }
    }

    printf("Exiting...\r\n");
	}
	else
	  printf("Fatal error, unable to initialize since something chandler"
			  "did is wrong\r\n");

}
