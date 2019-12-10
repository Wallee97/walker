#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/inout.h>


#include "acl8112dadriver.h"

void
ACL_8112_DA_Out(ACL_8112_DA *buff)
{
  int i;

  for(i=0;i<2;i++){
    buff->lsb[i] = ((buff->buffer[i]) & 0xff);
    buff->msb[i] =(((buff->buffer[i]) & 0xf00) >> 8);

    out8(buff->base + 2*i, buff->lsb[i]);
    out8(buff->base+1 + 2*i, buff->msb[i]);
  }
  out8(buff->base+0x10,0x0);
}



