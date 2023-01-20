
//#include <../../work/khepera4_development/libkhepera-2.1/src/khepera.h>
//#include <../../work/khepera4_development/libkhepera-2.1/src/knet.h>

#include "../libkhepera-2.1/src/khepera.h"
#include "cgripperI2C.c"
//#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/template/cgripperI2C.c>

// Error Log
FILE *f;

void error_Log(char* fileline){
	char* newline = "\r\n";
	char *line = (char*)malloc(1 + strlen(fileline) + strlen(newline));
	strcpy(line,fileline);
	strcat(line,newline);
	fprintf(f,"%s",line);
}

int initGripper( void )
{
  error_Log("Starting initialization for the cgripper_init()");
  if( cgripper_init() == 0 ){
	  error_Log("Gripper might have been initialized");
	  return 0;
  }
  error_Log("Gripper initialization might have failed");
  return -1;

}

int main( int arc, char *argv[])
{
  f = fopen("test.log","w");
  error_Log("Inzitializing Error Log File");

  printf("The Collective I2C Connection Test Program (C) The Collective WPI, January 18 22:02\r\n");

  error_Log("Calling initGripper");
  if(!initGripper())
  {
    error_Log("Initialized... maybe, might have wrong logic");
    error_Log("Please send via I2C!");

    unsigned short message = 13;

    while (1)
    { // Constantly send message

      // cgripper_Turret_Set_Max_Speed(message);
//      cgripper_Turret_Set_Max_Speed(KTeamGripper,0x14);
      cgripper_Gripper_Set_Position(message);
      printf("set gripper pos to: %hu \n", message);
      sleep(1);
      unsigned short pos = cgripper_Gripper_Get_Position();
      printf("read gripper pos as: %hu \n", pos);
      sleep(1);
    }

    error_Log("Exiting...");
	}
	else{
	  printf("Fatal error, unable to initialize since something Chandler did is wrong\r\n");
      error_Log("Fatal error, unable to initialize since something Chandler did is wrong");
	}



}
