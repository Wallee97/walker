#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include<hw/inout.h>

#include "acl8112dadriver.h"

void
ACL_8112_DA_Out(ACL_8112_DA *);

ACL_8112_DA*
daOpen(unsigned short port)
{
  ACL_8112_DA *buff;

  buff = (ACL_8112_DA *)malloc(sizeof(ACL_8112_DA));
  if (buff == NULL) {
    fprintf( stderr, "ERROR ACL-8112 D/A driver: Can not allocate enough memory\n");
    return NULL;
  }

  buff->base = 0x07c0/*port*/;

  fprintf("ACL-8112 D/A Initialized at base address 0x%x\n", buff->base);
  return buff;
}


void
daOut(ACL_8112_DA *buff, double *volt)
{
  buff->buffer[0] = volt_binary8112(*volt);
  buff->buffer[1] = volt_binary8112(*(volt+1));

  buff->outpVtage[0] = binary_volt8112(volt_binary8112(*volt));
  buff->outpVtage[1] = binary_volt8112(volt_binary8112(*(volt+1)));
  ACL_8112_DA_Out(buff);
}

void
zeroout(ACL_8112_DA *buff)
{
  /*static unsigned short zero = 0x000;*/
  static unsigned short zero = 0x011;
 
  printf("zeroout \n");
  
  buff->buffer[0] = volt_binary8112( 5 );
  buff->buffer[1] = volt_binary8112( 5 );

  buff->outpVtage[0] = zero;
  buff->outpVtage[1] = zero;
  
  ACL_8112_DA_Out(buff);
}

void
daClose(ACL_8112_DA *buff)
{
  free((void *)buff);
  return;
}






















