#define volt_binary8112(s) ((unsigned short)(s * 409.6))
 /* 4096*Vout/-(-10)   (outp : 0�`10[V])*/

#define binary_volt8112(s) ((float)(s / 409.6))
 /* -(-10)*DAn/4096   (outp : 0�`10[V])*/


typedef struct
{
  unsigned short base;
  unsigned short msb[2];
  unsigned short lsb[2];
  unsigned short buffer[2];
  double outpVtage[2];
} ACL_8112_DA;


extern ACL_8112_DA *daOpen(unsigned short);
extern void daOut(ACL_8112_DA *, double *);
extern void zeroout(ACL_8112_DA *);
extern void daClose(ACL_8112_DA *);

  
  






