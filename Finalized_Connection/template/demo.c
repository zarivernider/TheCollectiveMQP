
//#include <../../work/khepera4_development/libkhepera-2.1/src/khepera.h>
//#include <../../work/khepera4_development/libkhepera-2.1/src/knet.h>

#include "../src/khepera.h"
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
  // error_Log("Starting initialization for the cgripper_init()");
  if( cgripper_init() == 0 ){
	  // error_Log("Gripper might have been initialized");
	  return 0;
  }
  // error_Log("Gripper initialization might have failed");
  return -1;
}

int main( int arc, char *argv[])
{
  f = fopen("test.log","w");
  error_Log("Inzitializing Error Log File");

  printf("The Collective I2C Connection Test Program (C) The Collective WPI, March 21 21:04\r\n");

  // error_Log("Calling initGripper");
  if(!initGripper())
  {
    // error_Log("Initialized... maybe, might have wrong logic");
    // error_Log("Please send via I2C!");

    unsigned short message = 13;

    // set_turret_zero();
    // sleep(1);
    // set_EEPROM();
    // sleep(1);
    // printf("done setting zero \n");
    // setTurretSpeed(-500);
    // printf("done setting speed");

    // while (1)
    // { // Constantly send message

      // cgripper_Turret_Set_Max_Speed(message);
//      cgripper_Turret_Set_Max_Speed(KTeamGripper,0x14);
    //   cgripper_Gripper_Set_Position(message);
    //   printf("set gripper pos to: %hu \n", message);
    //   sleep(1);
      // unsigned short pos = cgripper_Gripper_Get_Position();
      // printf("read gripper pos as: %hu \n", pos);
      // sleep(1);
      
        //cgripper_Turret_Set_Max_Tolerance(141);
        
        // setKi(0);
        // setKp(1331);
        // rotateTurret();
        // int i = 0;
        // unsigned short pos = cgripper_Turret_Get_Position();
        // while(1){
        //    i = 0;
        //    setTurretPosition(22345); // 180deg
        //    while(i < 12){
        //      pos = cgripper_Turret_Get_Position();
        //      printf("read turret position as: %hu \n", pos);
        //      sleep(1);
        //      i++;
        //    }
        //    i = 0;
        //    setTurretPosition(0); // 180deg
        //    while(i < 12){
        //      pos = cgripper_Turret_Get_Position();
        //      printf("read turret position as: %hu \n", pos);
        //      sleep(1);
        //      i++;
        //    }
        // }

        // cgripper_Turret_Set_Autotrim(0x01);
        // sleep(2);
        // cgripper_Set_EEPROM(0x01);
        // sleep(2);

        // cgripper_Modularity_Test();
        cgripper_Open_Gripper();
        setAllGreen();
        sleep(2);

        
        // unsigned short degpos = pos*360/44690;
        // printf("read turret deg position as: %hu \n", degpos);
        // sleep(1);
        // testLED(); // is good
        // printf("done testing LEDs \n");
        // sleep(2);
        // unsigned short paraForce = cgripper_ForceSensor_Get__Parallel_Force(); // probably shouldn't be 0
        // printf("read parallel force sensor as: %hu \n", paraForce);
        // sleep(2);
        // unsigned short perpForce = cgripper_ForceSensor_Get__Perpendicular_Force(); // is good
        // printf("read perpendicular force sensor as: %hu \n", perpForce);
        // sleep(2);
        
        // rotateTurret();
        // printf("done rotating turret \n");
        // sleep(1);
        // usleep(250000);
        // stop();
        // sleep(8);

    // }

    // error_Log("Exiting...");
	}
	else{
	  printf("Fatal error, unable to initialize since something Chandler did is wrong\r\n");
      // error_Log("Fatal error, unable to initialize since something Chandler did is wrong");
	}



}
