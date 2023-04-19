/*-------------------------------------------------------------------------------
 * Project	 :	command line i2c test utility
 * File		 :	i2ctest.c
 * Author	 :	Yves Piguet, 2004
 */

#include <stdlib.h>
#include <stdio.h>
#include "../src/i2ccom.h"

int i2c_read8( i2c_t * i2c , 
	       i2c_dev_t dev , 
	       unsigned char reg , 
	       unsigned char *val )
{
  

  return i2c_lltransfer( i2c , dev , &reg , 1 , val , 1 );
}

int i2c_read16( i2c_t * i2c ,
		i2c_dev_t dev ,
		unsigned char reg ,
		unsigned short *val )
{
  return i2c_lltransfer( i2c , dev , &reg , 1 , 
			 (unsigned char *)val , 2 );
}

int i2c_read32( i2c_t * i2c ,
		i2c_dev_t dev ,
		unsigned char reg ,
		unsigned long *val )
{
  return i2c_lltransfer( i2c , dev , &reg , 1 , 
			 (unsigned char *)val , 4 );
}


int i2c_write8( i2c_t * i2c ,
		i2c_dev_t dev ,
		unsigned char reg ,
		unsigned char val )
{
  unsigned char buf[2];

  buf[0] = reg;
  buf[1] = val;

  return i2c_llwrite( i2c , dev , buf , 2 );
}

int i2c_write16( i2c_t * i2c ,
		 i2c_dev_t dev ,
		 unsigned char reg ,
		 unsigned short val )
{
  unsigned char buf[3];

  buf[0] = reg;
  buf[1] = val;
  buf[2] = (val>>8);

  return i2c_llwrite( i2c , dev , buf , 3 );
}

int i2c_write32( i2c_t * i2c ,
		 i2c_dev_t dev ,
		 unsigned char reg ,
		 unsigned long val )
{
  unsigned char buf[5];

  buf[0] = reg;
  buf[1] = val;
  buf[2] = (val>>8);
  buf[3] = (val>>16);
  buf[4] = (val>>24);

  return i2c_llwrite( i2c , dev , buf , 5 );
}
