
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/inout.h>

#include "hpc-ctrdriver.h"

HPC_CTR *
HPC_CTR_Init(HPC_CTR *);
long *
HPC_CTR_Read_Data(HPC_CTR *);

HPC_CTR *
cntOpen(unsigned short port) /* 0x0320 or 0x0340 */
{
  HPC_CTR *buff;
  
  buff = (HPC_CTR *)malloc(sizeof(HPC_CTR));
  if (buff == NULL) {
    fprintf(stderr, "ERROR HPC-CTR driver: Can not allocate enough memory\n");
    return NULL;
  }

  buff->base = port;
  
  buff = HPC_CTR_Init(buff);
  
  fprintf(stderr, "HPC-CTR Initialized at base address 0x%x\n", buff->base);

  return buff;
}

long *
cntRead(HPC_CTR *buff)
{
  return HPC_CTR_Read_Data(buff);
}

void
cntClose(HPC_CTR *buff)
{
  free((void *)buff);
  return;
}

HPC_CTR *
HPC_CTR_Init(HPC_CTR *buff)
{
  /* ctr off & clear */
  out8(buff->base+0x8,0x0f);  out8(buff->base+0x9,0x0f);

  /* 外部入力使用せず */
  out8(buff->base+0xa,0x00);  out8(buff->base+0xb,0x00);

  /*xch:UP/DOWNモード, ych,zch,uch:4逓倍 */
  /*  out8(buff->base+0xc,0x00);  out8(buff->base+0xd,0x10);
  out8(buff->base+0xe,0x03);  out8(buff->base+0xf,0x03);*/

  /*xch, ych, zch, uch 4逓倍　＋　カウント極性反転*/
  out8(buff->base+0xc,0x0b);  out8(buff->base+0xd,0x0b);
  out8(buff->base+0xe,0x0b);  out8(buff->base+0xf,0x0b);
  
  /* status2 報告せず */
  out8(buff->base+0x10,0x00);  out8(buff->base+0x11,0x00);

  /* timer off */
  out8(buff->base+0x1a,0x10);

  /* operation port off */
  out8(buff->base+0x1e,0x00);

  /* all ctr on */
  out8(buff->base+0x8,0x00);  out8(buff->base+0x9,0x00);

  return buff;
}

long *
HPC_CTR_Read_Data(HPC_CTR *buff)
{
  int i;
  unsigned short data;

  out8(buff->base+0x1e,0x40);    /* Latch All Channel */

  buff->buffer[0] = 0x0L;
  buff->buffer[1] = 0x0L;
  buff->buffer[2] = 0x0L;
  buff->buffer[3] = 0x0L;

  for(i=0;i<2;i++){
    /* read xch (zch) */

    out8(buff->base+0xa+i,0x01);     /* select xch (zch) */

    data = in8(buff->base+0x0+i);    /* read 1st byte (lsb) */
    buff->buffer[0+2*i] |= data;		          /* store it */

    data = in8(buff->base+0x2+i);    /* read 2nd byte */
    buff->buffer[0+2*i] |= (data << 8);		          /* store it */


    data = in8(buff->base+0x4+i);    /* read 3rd byte */
    buff->buffer[0+2*i] |= (data << 16);	         /* store it */
    
    data = in8(buff->base+0x6+i);    /* read 4th byte (msb) */
    buff->buffer[0+2*i] |= (data << 24);	        /* store it */

    
    /* read ych (uch) */

    out8(buff->base+0xa+i,0x02);
    
    data = in8(buff->base+0x0+i);
    buff->buffer[1+2*i] |= data;	
    
    data = in8(buff->base+0x2+i); 
    buff->buffer[1+2*i] |= (data << 8);	

    data = in8(buff->base+0x4+i); 
    buff->buffer[1+2*i] |= (data << 16);
    
    data = in8(buff->base+0x6+i); 
    buff->buffer[1+2*i] |= (data << 24);
  }

  
  /* Latch 解除 */
  out8(buff->base+0x1e,0x00);
  out8(buff->base+0x1f,0x00);
  
  return buff->buffer;
}

