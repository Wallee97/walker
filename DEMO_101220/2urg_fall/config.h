/**********************/
/*    \‘¢‘Ì‚Ì’è‹`    */
/**********************/
struct params {
  int trig;
  int mode;
  double desVel[2];
  double desTime;
  double desVol[2];
  double desVelocity;
  double desOmega;
  double desR;
  double deltaDamper[4];
  double deltaMass[2];
};

struct status {
  double pos[3];
  double vel[3];
  double acc[3];
  double omega;
  double velocity;
  double dmp[4];
  double mass[2];
  double Rad;
  double tilt[2];
};

struct path {
  double pos[3][6];
  double vel[3][5];
  double acc[3][4];
  double time;
};
