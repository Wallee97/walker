#include <stdio.h>
#include <vxWorks.h>
#include <sysLib.h>

#include "acl8112addriver.h"


ACL_8112_AD *
ACL_8112_AD_Init(ACL_8112_AD *buff)
{
  int i;

  /* A/D Channel Multiplexer Register */
  /* SELECTION : ch0-diffrential input */

  /* A/D Renge Control Register */
  /* SELECTION : Unipolar, 0V-10V */
  sysOutByte(buff->base+9,0x4);

  /* A/D Operation Mode Control Register */
  /* SELECTION : software trigger and program polling */
  sysOutByte(buff->base+11,0x1);
  
  for(i=0; i<8; i++){
    buff->buffer[i] = 0x0000;
    buff->lsb[i] = 0x0000;
    buff->msb[i] = 0x0000;
  }; 
  
  return buff;
}


float *
ACL_8112_AD_In(ACL_8112_AD *buff, unsigned short ch)
{
  unsigned short MSB;
  float *addr;
  long l;
  
  /* A/D Renge Control Register */
  /* SELECTION : Unipolar, 0V-10V */
  sysOutByte(buff->base+9,0x04);   /* 4:0->+10  1:-0.5->+0.5  0:-5->+5 */
  if(ch == 5) sysOutByte(buff->base+9,0x00);

  /* A/D Channel Multiplexer Register */
  switch(ch){
  case 0: sysOutByte(buff->base+10, /*0x10*/0x30);break;
  case 1: sysOutByte(buff->base+10, /*0x11*/0x31);break;
  case 2: sysOutByte(buff->base+10, /*0x12*/0x32);break;
  case 3: sysOutByte(buff->base+10, /*0x13*/0x33);break;
  case 4: sysOutByte(buff->base+10, /*0x14*/0x34);break;
  case 5: sysOutByte(buff->base+10, /*0x15*/0x35);break;
  case 6: sysOutByte(buff->base+10, /*0x16*/0x36);break;
  case 7: sysOutByte(buff->base+10, /*0x17*/0x37);break;
  };

  for(l=0; l<4000; l++);        
  /* Software Trigger Register */
  sysOutByte(buff->base+12, 0x0f);   /* change A to D starting */

  /* polling */
  buff->lsb[ch]=ZERO; buff->msb[ch]=ZERO;
  buff->buffer[ch]=ZERO; MSB=ZERO;
  while( (sysInByte(buff->base+5) & DRDY) == DRDY );
  MSB = sysInByte(buff->base+5);
  buff->msb[ch] = ~((~MSB) | DRDY);
  MSB = ((buff->msb[ch])<<8);
  buff->lsb[ch] = sysInByte(buff->base+4);

  buff->buffer[ch] = binary_10_8112((MSB | (buff->lsb[ch])));
  /*MSB = ZERO;*/
  addr = (buff->buffer) + ch;
  return addr;
} 


