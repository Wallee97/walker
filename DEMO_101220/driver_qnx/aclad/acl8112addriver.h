#define DRDY ((unsigned short) 0x10)
#define ZERO ((unsigned short) 0x00) 
#define CH0 ((unsigned short) 0x00)
#define CH1 ((unsigned short) 0x01)
#define CH2 ((unsigned short) 0x02)
#define CH3 ((unsigned short) 0x03)
#define CH4 ((unsigned short) 0x04)
#define CH5 ((unsigned short) 0x05)
#define CH6 ((unsigned short) 0x06)
#define CH7 ((unsigned short) 0x07)

#define binary_10_8112(s) ((float)(s / 409.6))
 /* 10*ADn/4096 (range: 0�`10[V]) */


typedef struct
{
  unsigned short base;
  float buffer[8]; /*differential input*/
  unsigned short lsb[8];
  unsigned short msb[8];
  int j[8]; 
} ACL_8112_AD;


extern ACL_8112_AD *adOpen(unsigned short);

extern float *adRead(ACL_8112_AD *, unsigned short ch);

extern void adClose(ACL_8112_AD *);

  
  
