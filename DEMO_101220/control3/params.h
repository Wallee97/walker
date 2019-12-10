 /************/
/* params.h */ 
/************/

/*#define PI   (3.14159265358979323846)*/
#define PI   (3.1415)
#define Grav (9.80665)

#define rad2deg(c)   (180.*(c)/  PI)
#define deg2rad(c)   (  PI*(c)/180.)

#define ON   (1)
#define OFF  (0)

#define on   (1)
#define off  (0)

#define right (0)
#define left (1)

#define fx   (0)
#define fy   (1)
#define mz   (2)

#define X    (0)
#define Y    (1)
#define TH   (2)
#define V    (3)
/* #define fb   (4) */
/* #define nb   (5) */

/* #define dis  (0) */
/* #define ang  (1) */
/* #define type (2) */
/* #define XL   (3) */
/* #define YL   (4) */
/* #define XR   (5) */
/* #define YR   (6) */
/* #define XG   (7) */
/* #define YG   (8) */
/* #define Vh   (9) */

#define dis  (0)
#define ang  (1)
#define type (2)
#define XG   (3)
#define YG   (4)
#define xHip (5)
#define yHip (6)


#define M    (0)
#define J    (1)

#define r    (0)
#define l    (1)

#define alpha (0)
#define beta  (1)

#define horizontal   (3)
#define obstacle     (1)
#define step         (2)
#define unknown      (0)
/*--------------- timer --------------------------------------------------*/

#define FREQ                   (500)                 /* Hz  */
#define TICKS               (1./FREQ)                /* sec */
#define logsFREQ                (10)                 /* Hz  */
#define logsTICKS       (1./logsFREQ)                /* sec */
#define logdFREQ                (1)                  /* Hz  */
#define logdTICKS       (1./logdFREQ)                /* sec */

#define logsMax               (65535)

/*--------------- I/O Board Base Address ---------------------------------*/

#define ACL8112_Addr    0x0380        /* DA/AD adress      */
#define HPCCTR_Addr     0x0320        /* Counter Adress */

/*--------------- Gear_rate & Torqconst ----------------------------------*/

#define Gear_Rate     (1./2.5)
#define Torqconst_0     (3.1103)
#define Torqconst_1     (3.1103)
/*--------------- Count -> Angle (rad) -----------------------------------*/

#define count2angle_R(c)  ((c) * 2. * PI / 3600. / 4. * Gear_Rate)
#define count2angle_L(c)  (-(c) * 2. * PI / 3600. / 4. * Gear_Rate)
/*--------------- Tilt Sensor Output[V] -> Angle [rad] -------------------*/

#define volt2angle(c)  (((c)-2.59)*PI/(0.0633*180.))
/*--------------- Radius & Tred ------------------------------------------*/

#define MC_r   (0.14885/2.)        /* Radius of wheel */
#define MC_T   (0.600)       /* Thread */ 
/* #define H      (0.531) /\* length of TATE  (sick‚Ìİ’uêŠ)*\/ */
/* #define H      (1.7)   /\* length of TATE*\/ */
/* #define H      (1.5)       /\* length of TATE *\/ */

/* #define K        (0.300) */
#define H_m      (0.400)
#define MC_A   (deg2rad(48.))   

/*--------------- Inertia ------------------------------------------------*/

double Inertia[2] = {  0.0931,  0.0931 };

/*--------------- Damper  ------------------------------------------------*/

double Damper[2] = { 0.055, 0.055 };

/*--------------- Desired Internal Force ---------------------------------*/

const double Internal_force[3] = { 0.0, 0.0, 0.0 };

/*--------------- Weight  ------------------------------------------------*/

double M_w = { 45.0 };
double M_w_h = { 90.0 };

/*--------------- Gain ---------------------------------------------------*/
/*- PI‚ÌƒQƒCƒ“ -*/

double Kp[2] = { 175. , 145.};
double Ki[2] = { 50. , 30. };

/*--------------- Constant value for Trace Control -----------------------*/
/*double K_omega = { 70.};*/

double K_omega = { 100.};
double K_theta = { 1620. };
double K_eta_d = { 12600. };

/*double K_v = { 500000.};*/
/*double K_th = { 500000. };*/

double K_v = {1000.};
double K_th = {40000.};

double r_off = { 20.};

/*
double K_omega={30.};
double K_theta={310.};
double K_eta_d={1100.};
*/
/*---------------- Feedback gain -----------------------------------------*/

double k_f_b = { 500. };
double k_t_b = { 500. };

/*---------------- Contant value for Avoidance Control -------------------*/
double eta = { 14.};     /*@d‚İŒW”ƒÅ@*/
//double eta2 = { 250.};
double rou0 = { 0.6 };  /*  ³‚Ì’è”ƒÏ_0
						   ƒƒ{ƒbƒg‚©‚çƒÏ_0ˆÈã—£‚ê‚½êŠ‚Å‚Íƒ|ƒeƒ“ƒVƒƒƒ‹‚O
						   */


/*-----------------³‹K•ª•z‚Ì•½‹Ï‚Æ•ªU‚Ì’l-------------------------------*/

#define mu_x   (16.787)  //•½‹Ï ˜_•¶‚Æxy‚ª‹t!!
#define mu_y   (292.512)
#define v_x   (19.197)   //•Î·
#define v_y   (24.715)
#define v2_x   (368.548) //•ªU
#define v2_y   (610.845)
#define ro   (-0.102) //‘ŠŠÖŒW”



/* #define mu_x   (5.754405)  //•½‹Ï */
/* #define mu_y   (318.181443)  */
/* #define v_x   (21.010650)   //•Î· */
/* #define v_y   (59.402445)  */
/* #define v2_x   (368.548) //•ªU */
/* #define v2_y   (3528.650449)  */
/* #define ro   (-0.215038) //‘ŠŠÖŒW” */
#define P_l   (0.000001)  //è‡’l




