#include <vxWorks.h>
#include <stdio.h>
#include <stdlib.h>
#include <sysLib.h>
#include <logLib.h>

#include "hpc-ctrdriver.h"

/* #include "hpc-ctrdriver.h" */
HPC_CTR *
HPC_CTR_Init(HPC_CTR *);
long *
HPC_CTR_Read_Data(HPC_CTR *);


HPC_CTR *
cntOpen(unsigned short port /* 0x0320 or 0x0340 */)
{
  HPC_CTR *buff;
  
  buff = (HPC_CTR *)malloc(sizeof(HPC_CTR));
  if (buff == NULL) {
    logMsg("ERROR HPC-CTR driver: Can not allocate enough memory\n",0,0,0,0,0,0);
    return NULL;
  }

  buff->base = port;
  
  buff = HPC_CTR_Init(buff);
  
  logMsg("HPC-CTR Initialized at base address 0x%x\n", buff->base,0,0,0,0,0);

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




