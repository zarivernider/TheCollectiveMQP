
//#include <../../work/khepera4_development/libkhepera-2.1/src/khepera.h>
//#include <../../work/khepera4_development/libkhepera-2.1/src/knet.h>

#include "../libkhepera-2.1/src/khepera.h"
// #include "cgripperI2C.c"
#include <time.h>
//#include </home/midnightpegasus/khepera4_development/libkhepera-2.1/template/cgripperI2C.c>

// Error Log
FILE *f;
i2c_t i2c;
char *devpath = NULL;// might default to correct devpath, but unsure

void error_Log(char* fileline){
	char* newline = "\r\n";
	char *line = (char*)malloc(1 + strlen(fileline) + strlen(newline));
	strcpy(line,fileline);
	strcat(line,newline);
	fprintf(f,"%s",line);
}

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
    // Storing start time
    clock_t start_time = clock();
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds);
}

void testLED() {
  int addr = 0x15;
  i2c_write16(&i2c, addr, 0x16, 0x6DB6);
  delay(10);
  i2c_write16(&i2c, addr, 0x17, 0xDB6D);
  delay(10);
  i2c_write16(&i2c, addr, 0x18, 0XB6DB);
  delay(10);
  i2c_write16(&i2c, addr, 0x19, 0X6DB6);
  delay(10);
  i2c_write16(&i2c, addr, 0x19, 0x0001);
  delay(10);
}

int main( int arc, char *argv[])
{
  f = fopen("test.log","w");
  error_Log("Inzitializing Error Log File");

  printf("The Collective I2C Connection Test Program (C) The Collective WPI, January 18 22:02\r\n");

  error_Log("Calling initGripper");
  fprintf(stderr, "Opening i2c device %s\n", devpath);

  if (i2c_open(&i2c, devpath) < 0) {
    error_Log("Opened I2C");
  }
  
  
  error_Log("Initialized... maybe, might have wrong logic");
  error_Log("Please send via I2C!");

  while (1)
  { 
    testLED;
    printf("Sending LED\n");
  }

  error_Log("Exiting...");
	
	else{
	  printf("Fatal error, unable to initialize since something Chandler did is wrong\r\n");
      error_Log("Fatal error, unable to initialize since something Chandler did is wrong");
	}



}
