
//#include <../../work/khepera4_development/libkhepera-2.1/src/khepera.h>
//#include <../../work/khepera4_development/libkhepera-2.1/src/knet.h>

#include "../src/khepera.h"
#include "cgripperI2C.c"
//#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/template/cgripperI2C.c>

// Error Log
FILE *f;

static knet_dev_t * dsPic; // robot pic microcontroller access

void error_Log(char* fileline){
	char* newline = "\r\n";
	char *line = (char*)malloc(1 + strlen(fileline) + strlen(newline));
	strcpy(line,fileline);
	strcat(line,newline);
	fprintf(f,"%s",line);
}

/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig ) 
{
  
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  
  kb_change_term_mode(0); // revert to original terminal if called
  
  exit(0);
}

int initGripper( void )
{
  // error_Log("Starting initialization for the cgripper_init()");
  if( cgripper_init() == 0 ){
	  // error_Log("Gripper might have been initialized");
	  return 0;
  }
  // error_Log("Gripper initialization might have failed");
  return -1;
}

void destroy(void){
  /* Stop wheels */
   kh4_set_speed(0, 0, dsPic);
   kh4_SetMode(kh4RegIdle, dsPic );
   /* Switch LEDs off */
   kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0, dsPic);
   /* Switch ultrasound sensor off */
   kh4_activate_us(0, dsPic);
}

void initialize(void){
  /* Initialize Khepera */
  if(kh4_init(0,NULL) != 0) {
    printf("Error initializing the Khepera IV subsystem (kh4_init)\n");
  }
  /* open robot socket and store the handle in its pointers */
  dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );

  if ( dsPic==NULL)
  {
    printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  }	else {
    printf("\nConnected with Kh4 dsPic\n\n");
  }

  kh4_SetMode(kh4RegSpeedProfile, dsPic);

}

int main( int arc, char *argv[])
{
  printf("The Collective I2C Connection Test Program (C) The Collective WPI, April 17\r\n");
  initialize();
  // error_Log("Calling initGripper");
  if(!initGripper())
  {
    cgripper_Open_Gripper();
    setAllOff();
    sleep(2);
    destroy();
	}
	else{
	  printf("Fatal error, unable to initialize I2C Connection\r\n");
    destroy();
	}



}
