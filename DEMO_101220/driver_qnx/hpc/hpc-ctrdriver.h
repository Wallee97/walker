
typedef struct
{
  unsigned short base;
  long buffer[4];
} HPC_CTR;

extern HPC_CTR *cntOpen(unsigned short);
extern long *cntRead(HPC_CTR *);
extern void cntClose(HPC_CTR *);
