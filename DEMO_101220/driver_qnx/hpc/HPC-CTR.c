#include <stdio.h>

#include <hpc-ctrdriver.h>

HPC_CTR *
HPC_CTR_Init(HPC_CTR *buff)
{
  /* ctr off & clear */
  sysOutByte(buff->base+0x8,0x0f);  sysOutByte(buff->base+0x9,0x0f);

  /* �O�����͎g�p���� */
  sysOutByte(buff->base+0xa,0x00);  sysOutByte(buff->base+0xb,0x00);

  /* xch:UP/DOWN���[�h, ych,zch,uch:4���{ */
  sysOutByte(buff->base+0xc,0x00);  sysOutByte(buff->base+0xd,0x10);
  sysOutByte(buff->base+0xe,0x03);  sysOutByte(buff->base+0xf,0x03);

  /* status2 �񍐂��� */
  sysOutByte(buff->base+0x10,0x00);  sysOutByte(buff->base+0x11,0x00);

  /* timer off */
  sysOutByte(buff->base+0x1a,0x10);

  /* operation port off */
  sysOutByte(buff->base+0x1e,0x00);

  /* all ctr on */
  sysOutByte(buff->base+0x8,0x00);  sysOutByte(buff->base+0x9,0x00);

  return buff;
}

long *
HPC_CTR_Read_Data(HPC_CTR *buff)
{
  int i;
  unsigned short data;

  sysOutByte(buff->base+0x1e,0x40);    /* Latch All Channel */

  buff->buffer[0] = 0x0L;
  buff->buffer[1] = 0x0L;
  buff->buffer[2] = 0x0L;
  buff->buffer[3] = 0x0L;

  for(i=0;i<2;i++){
    /* read xch (zch) */

    sysOutByte(buff->base+0xa+i,0x01);     /* select xch (zch) */

    data = sysInByte(buff->base+0x0+i);    /* read 1st byte (lsb) */
    buff->buffer[0+2*i] |= data;		          /* store it */

    data = sysInByte(buff->base+0x2+i);    /* read 2nd byte */
    buff->buffer[0+2*i] |= (data << 8);		          /* store it */


    data = sysInByte(buff->base+0x4+i);    /* read 3rd byte */
    buff->buffer[0+2*i] |= (data << 16);	         /* store it */
    
    data = sysInByte(buff->base+0x6+i);    /* read 4th byte (msb) */
    buff->buffer[0+2*i] |= (data << 24);	        /* store it */

    
    /* read ych (uch) */

    sysOutByte(buff->base+0xa+i,0x02);
    
    data = sysInByte(buff->base+0x0+i);
    buff->buffer[1+2*i] |= data;	
    
    data = sysInByte(buff->base+0x2+i); 
    buff->buffer[1+2*i] |= (data << 8);	

    data = sysInByte(buff->base+0x4+i); 
    buff->buffer[1+2*i] |= (data << 16);
    
    data = sysInByte(buff->base+0x6+i); 
    buff->buffer[1+2*i] |= (data << 24);
  }

  
  /* Latch ���� */
  sysOutByte(buff->base+0x1e,0x00);
  sysOutByte(buff->base+0x1f,0x00);
  
  return buff->buffer;
}








