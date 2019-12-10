#include <stdio.h>
#include <stdlib.h>
#include <hw/inout.h>

#include "acl8112addriver.h"


ACL_8112_AD *
ACL_8112_AD_Init(ACL_8112_AD *buff);

float *
ACL_8112_AD_In(ACL_8112_AD *buff, unsigned short ch);


ACL_8112_AD*
adOpen(unsigned short port)
{
  ACL_8112_AD *buff;

  buff = (ACL_8112_AD *)malloc(sizeof(ACL_8112_AD));
  if (buff == NULL) {
    fprintf(stderr, "ERROR ACL-8112 A/D driver: Can not allocate enough memory\n");
    return NULL;
  }

  buff->base = port;

  buff = ACL_8112_AD_Init(buff);
 
  fprintf(stderr,"ACL-8112 A/D Initialized at base address 0x%x\n", buff->base);

  return buff;
}


float *
adRead(ACL_8112_AD *buff, unsigned short ch)
{
  return  ACL_8112_AD_In(buff, ch);
}


void
adClose(ACL_8112_AD *buff)
{
  free((void *)buff);
  return;
}










