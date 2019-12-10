 /* main.c: Program of Walking Helper for QNX                                  */
/*                                                                            */
/* CAUTION  : THIS PROGRAM IS ONLY FOR  " Q N X "                             */
/*                                                                            */
/* Auther   : Asami Hara & Asami Muraki                                       */
/* Thanks   : Takahiro Baba(wrote base program                                */
/* Date     : 2004/                                                           */
/* Version  : 2                                                               */
/* comment  : Obstacle Avoidance program using SICK LMS                       */
/* See Also : config.h   (Definition of Struct)                               */
/*          : params.h   (Parameter Definition)                               */
/*          : cp.c       (Command Processing Module)                          */
/*          : timer.c    (QNX Timer Module)                                   */
/*          : timer.h    (QNX Timer Definitions)                              */
/*          : ./driver_qnx/aclda/DA_driver.c  (DA converter Driver Module)     */
/*          : ./driver_qnx/aclda/acl8112dadriver.h (DA converter Driver Header)*/
/*          : ./driver_qnx/hpc/hpc-ctrdriver.h (Counter Driver Header)        */
/*          : ./driver_qnx/aclad/AD_driver.c  (AD converter driver module)    */
/*          : ./driver_qnx/aclad/acl8112addriver.h (AD converter Driver Header)*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/dispatch.h>
#include <sys/siginfo.h>
#include <sys/neutrino.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/wait.h>

#include "params.h"
#include "config.h"
#include "function.h"

#include "../driver_qnx/aclda/acl8112dadriver.h"
#include "../driver_qnx/hpc/hpc-ctrdriver.h"
#include "../driver_qnx/aclad/acl8112addriver.h"
#include "timer.h"

#define TRUE  1
#define FALSE 0

#define ERROR EXIT_FAILURE
#define OK    EXIT_SUCCESS

struct status Cur_e, pre_Cur_e, des_Cur_e;        /* absolute coordinate-system      */
struct status Cur_e_c;                         /* control position in absolute coordinate-system*/
struct status Cur_j, pre_Cur_j, des_Cur_j, pre_des_Cur_j;      /* joint coordinate-system */
struct status Cur_w, pre_Cur_w, des_Cur_w;        /* wheels velocity (Cur_w.vel)     */

enum mode {nop, current, vel_control, direct_control, trace_control, trace_control_2,
		   ad_test, slope_control,
           trace_control_3, trace_control_4, trace_control_5, trace_control_6,
		   model_control_2, model_control_3, model_control_cas,
		   model_control_4, model_control_5, model_control_only, model_control_only2,
		   avoid_control_1,
		   avoid_control_2, avoid_control_3, avoid_control_4, avoid_control_5,
		   lms_control, demo_control, obs_control };

/* Definition of Running Mode Enumeration                                     */
/*      nop        : No Operand/Operation Mode                                */

struct params Motorp;

ACL_8112_DA *da;                  /* D/A Board                       */
HPC_CTR *cnt;                    /* Counter Board                   */
ACL_8112_AD *ad;                  /* A/D Board                       */

int ctrlTaskID = 0;                     /* Task of Control                 */

int ctrlEndFlag = 0;

int maintrig = OFF;
int ctrltrig = OFF;
int logstrig = OFF;
unsigned long logcnt = 0;
float logbuff[17][logsMax];

int logsTaskID = 0;

unsigned short ch0 = 0, ch1 = 1, ch2 = 2, ch3 = 3;

double Ki_control[2];

double er[3];

double K_eta;
double PR, PQ;
double des_th;

double f_b, t_b;                         /* ���i�u���[�L�́C��]�p�u���[�L�� */

double H;                               /* ��\�_�܂ł̋��� */
double DH;                              /* �f���p�̑�\�_�܂ł̋���*/

/* Parameter for Avoidance Control */

double F_R={0.};                         /* Repulsive Force (��)�˗�          */  
double TH_R;                            /* Repulsive Force�@�̕���           */
double P_q;                              /* ��Q����walker�̐���_�Ƃ̍ŏ�����*/
double Cross_p[2];                       /* Walker���猩����Q���Ɉ�ԋ߂��_  */  
double F_r[3];                           /* Repulsive Force (����)�˗�        */
extern thread_pool_t * cp (int argc, char **argv);

/*double torq[2];*/

/******************************************************************************/
/*  another process variables                                                 */

#define MYPORT 3490
#define BACKLOG 10
unsigned short sickdata[361]={0};
unsigned short rmsg[1]={1};
//unsigned short  mindata[2] = {0};
double mindata[2] = {0.};
double mindata_d[3]={0.};
//double CurData[4]={0.};
int sockfd, new_fd;
struct sockaddr_in my_addr;
struct sockaddr_in their_addr;
int sin_size;

/******************************************************************************/

/*                                                                            */
/*    create_sick ( void )                                                    */
/*                                                                            */
/*    --- explanation ---                                                     */
/*     called in init(void)                                                   */
/*                                                                            */
void create_sick(void)
{
   
	
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
		{
            perror("socket");
            exit(1);
        }
   
	my_addr.sin_family = AF_INET;         /* host byte order */
	my_addr.sin_port = htons(MYPORT);     /* short, network byte order */
	my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
	//bzero(&(my_addr.sin_zero), 8);        /* zero the rest of the struct */
	memset(&(my_addr.sin_zero),0, 8);        /* zero the rest of the struct */
	
	if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
		{
            perror("bind");
            exit(1);
        }
	

	if (listen(sockfd, BACKLOG) == -1) 
		{
            perror("listen");
            exit(1);
        }
	
	sin_size = sizeof(struct sockaddr_in);
        if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr,&sin_size)) == -1) 
	  {
	    perror("accept");
	    
	  }
	
	//printf("Server: got connection from %s\n",inet_ntoa(their_addr.sin_addr));
	
}

/*                                                                            */
/*    init ( void )                                                           */
/*                                                                            */
/*   --- explanation ---                                                     */
/*      init: initialization of all program.                                  */
/*         * Create Channel which used to receive commands                    */
/*         * Setup Timer (timer is now disable)                               */
/*         * initializeAll()                                                  */
/*                                                                            */
void 
init(void)
{
    ThreadCtl(_NTO_TCTL_IO,0);

    create_sick();
	
    if ((chid = ChannelCreate (0)) == -1){
        fprintf (stderr,"timer.c: couldn't create channel!\n");
        perror (NULL);
        exit (EXIT_FAILURE);
    }
    
    setupTimer();

    initializeAll();

}

/*                                                                            */
/*    initializeAll ( void )                                                  */
/*                                                                            */
/*    --- explanation ---                                                     */
/*      Initialization.                                                       */
/*         * initializeData()                                                 */
/*         * open device drivers                                              */
/*                                                                            */
/*    --- called ---                                                          */
/*      init()                                                                */
/*                                                                            */
/*    --- return ---                                                          */
/*      int ( OK / ERROR )                                                    */
/*                                                                            */
int
initializeAll(void)
{
  if(( da  = daOpen(ACL8112_Addr)) == NULL) return ERROR;
  if(( cnt = cntOpen(HPCCTR_Addr)) == NULL) return ERROR;
  if(( ad  = adOpen(ACL8112_Addr)) == NULL) return ERROR;
  //fprintf(stderr,"da %d\n",da);
  zeroout(da);
    
  initializeData();
    return(OK);
}

/*                                                                            */
/*    initializeData();                                                       */
/*                                                                            */
void
initializeData(void)
{

/* Current position of the robot                                              */
    Cur_e.pos[X]    = 0;
    Cur_e.pos[Y]    = 0;
    Cur_e.pos[TH]   = 0;
    
}
/*                                                                            */
/*    start ( void )                                                          */
/*                                                                            */
void
start(void)
{

    timer_settime (timerid, 0, &timer, NULL);
    ctrltrig = 1;
    ctrlEndFlag = 1;
    ctrlTask(&Motorp);

}


/*                                                                            */
/*    ctrlTask ( struct params *motor )                                       */
/*                                                                            */
/*    --- explanation ---                                                     */
/*      After clock signal(i.e. can take control semaphore),                  */
/*      take semaphore and excute control().                                  */
/*                                                                            */
/*    --- called ---                                                          */
/*      start()                                                               */
/*                                                                            */
int
ctrlTask(struct params *motor)
{
    int rcvid;
    MessageT msg;
    static unsigned long ticks = 0;
    
    ctrlEndFlag = 1;

    while(ctrltrig){
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if(rcvid == 0){

            if(!ctrlEndFlag){
                printf("ERROR : Control Task is out of time.\n        fin\n");
                fin();
                return (ERROR);
            }
    
            ctrlEndFlag = 0;

            control(motor);
            ticks++;
	    /*
	    if(logstrig){
	      if((ticks%(int)(logsTICKS*FREQ))==0)
		logsTask();
	    }*/
 
        }
    }
    
    return (EXIT_SUCCESS);
    
}

/*                                                                          */
/* logsTask( void )                                                         */
/*                                                                          */
/* -- explanation --                                                        */
/*  Excute logSave() logding data to file(save).                            */
/* -- called --                                                             */
/*  ctrlTask()                                                              */
/*
int
logsTask()
{
  logcnt = logSave();
}
*/
/* logSave( void )                                                           */

int
logSave(double buff[])
{
  int i;
  
  static unsigned long int logtime = 0;
 
  static double buffs[17] = {0.};
 
  for(i=0;i<17;i++){
  buffs[i]=buff[i];
  }
  
  logtime++;
  if(logtime<logsMax){
        for(i=0;i<17;i++){
                logbuff[i][logtime] = buffs[i];
       }
    }
  return(logtime);
}

/*  logWrite() */
void
logWrite(void)
{
    int i;
    FILE *file;

    printf("\n\tWait for minite...\n");
   
    if((file = fopen("data.dat","w")) == NULL){
        printf("file open error!\n");
        exit(1);
    }

    for(i=1;i<logcnt;i++){
       fprintf(file,
	 "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"
        ,i/10., logbuff[0][i], logbuff[1][i], 
         logbuff[2][i], logbuff[3][i], logbuff[4][i], logbuff[5][i], logbuff[6][i],
         logbuff[7][i], logbuff[8][i], logbuff[9][i], logbuff[10][i], logbuff[11][i], 
         logbuff[12][i], logbuff[13][i], logbuff[14][i], logbuff[15][i], logbuff[16][i]);
    } 

    fclose(file);
    printf("\n\tDatafile Save OK!\n\n");
}


/*                                                                            */
/*    control ( struct params *motor )                                        */
/*                                                                            */
/*    --- explanation ---                                                     */
/*      Start Control.                                                        */
/*      getCurrentStatus()                                                    */
/*      Switch control mode.                                                  */
/*         nop        : No Operand/Operation Mode                             */
/*         current    : Current Ctrl only                                     */
/*    --- called ---                                                          */
/*      ctrlTask()                                                            */
/*                                                                            */
void
control(struct params *motor)
{
    static struct params param;
    static int mode = nop;
    static int trig;

    memcpy(&param, motor, sizeof(struct params));

    if(param.trig){
        trig = TRUE ;
        motor->trig = FALSE;
    }
    else
        trig = FALSE;

    mode = param.mode;
 
    switch(mode){
    case current:
		CurCtrl(&param);
        break;
    case vel_control:
		VelCtrl(&param, trig);
		break;
	case ad_test:
		AD_test(&param, trig);
		break;
    case direct_control:
		DirCtrl(&param, trig); 
		break;
    case trace_control:
		TraCtrl(&param, trig);
		break;
    case trace_control_2:
		TraCtrl2(&param, trig);
		break;
    case trace_control_3:
		TraCtrl3(&param, trig);
		break;
    case trace_control_4:
		TraCtrl4(&param, trig);
		break;
    case trace_control_5:
		TraCtrl5(&param, trig);
		break;
    case trace_control_6:
		TraCtrl6(&param, trig);
		break;
    case model_control_2:
		ModelCtrl2(&param, trig);
		break;
    case model_control_3:
		ModelCtrl3(&param, trig);
		break;
    case model_control_cas:
		ModelCtrl3a(&param, trig);
		break;
    case model_control_4:
		ModelCtrl4(&param, trig);
		break;
    case model_control_5:
		ModelCtrl5(&param, trig);
		break;
    case model_control_only:
		ModelCtrlOnly(&param, trig);
		break;
	case model_control_only2:
		ModelCtrlOnly2(&param, trig);
		break;
    case avoid_control_1:
		AvoidCtrl1(&param, trig);
		break;
    case avoid_control_2:
		AvoidCtrl2(&param, trig);
		break;
    case avoid_control_3:
		AvoidCtrl3(&param, trig);
		break;
    case avoid_control_4:
		AvoidCtrl4(&param, trig);
		break;
    case avoid_control_5:
		AvoidCtrl5(&param, trig);
		break;
    case slope_control:
		SlopeCtrl(&param, trig);
		break;
    case lms_control:
		LMSAvoidCtrl(&param,trig);
		break;
    case demo_control:
		DEMOAvoidCtrl(&param,trig);
		break;
	case obs_control:
		OBSCtrl(&param,trig);
		break;	
    default:
        Nop();
        break;
    }
}

/* LMSAvoidCtrl                                                    */
/* command : lms(double dam[0], double dam[1])                     */
/*   �����m�[�g�T�@�Q��                                            */
void
LMSAvoidCtrl(struct params *param, int trig)
{
	static unsigned long ticks = 0;
	
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH]= param->deltaDamper[TH];
		printf("LMS & AvoidCtrl Start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

	if(ticks%100==0){
		get_sick_data();
	}
	
		getObstacleAbsoluteCoordinate();
		getNearestDistance();

		if(P_q<=rou0) {
			RepulsiveForce();
		}
		else {
			F_R = 0.0;
		}
	
		/*�i�s�����u���[�L��*/
 		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;

		/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 

		if(ticks%500==0){
			/* if(mindata_d[type]==step) printf("TYPE = STEP\n"); */
/* 			else if(mindata_d[type]==obstacle) printf("TYPE = OBSTACLE \n"); */
/* 			else printf("TYPE = UNKNOWN\n"); */
			printf("P_q= %f\nH=%f\n", P_q,H);
		}
		
		TorqCtrl();
		trig = 0;
		ticks++;
		return;
}

/*�f���p�v���O����*/
/* command : demo(double dam[0], double dam[1])*/
void
DEMOAvoidCtrl(struct params *param, int trig)
{
	static unsigned long ticks = 0;
	
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH]= param->deltaDamper[TH];
		printf("LMS & AvoidCtrl Start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

	if(ticks%20==0){
		get_sick_data();
	}
	
		getObstacleAbsoluteCoordinate();
		getNearestDistance();

		if(P_q<=rou0) {
			RepulsiveForce();
		}
		else {
			F_R = 0.0;
		}
	
		/*�i�s�����u���[�L��*/
 		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;

		/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 

		if(ticks%500==0){
			if(mindata_d[type]==step) printf("TYPE = STEP\n");
			else if(mindata_d[type]==obstacle) printf("TYPE = OBSTACLE \n");
			else printf("TYPE = UNKNOWN\n");
		}
		
		TorqCtrl();
		trig = 0;
		ticks++;
		return;
}
/* OBSCtrl                                                         */
/* command : obs(double dam[0], double dam[1])                     */
/* �L���X�^�[�������l��������Q����� & �d�͕⏞                   */
void
OBSCtrl(struct params *param, int trig)
{
	static unsigned long ticks = 0;
//	double mmm = 40.;
	
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH]= param->deltaDamper[TH];
		printf("LMS & AvoidCtrl Start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

	ADCtrl();

	if(ticks%20==0){
		get_sick_data();
	}
	
		getObstacleAbsoluteCoordinate();
		getNearestDistance();

		if(P_q<=rou0) {
			RepulsiveForce();
		}
		else {
			F_R = 0.0;
		}

        /*�i�s�����u���[�L��*/
		
	/* 	if(Cur_e.tilt[beta]<=0.&& Cur_e.velocity>=0.){ */
/* 			f_b = des_Cur_e.dmp[V] * Cur_e.velocity +mmm * Grav * sin(-Cur_e.tilt[beta])- F_R * cos(TH_R); */
		   
/* 	} */
/* 		if(Cur_e.tilt[beta]<=0 && Cur_e.velocity<0.){ */
/* 			f_b = -des_Cur_e.dmp[V] * Cur_e.velocity + mmm * Grav * sin(-Cur_e.tilt[beta])- F_R * cos(TH_R);  */
			
/* 		} */
/* 		if(Cur_e.tilt[beta]>0. && Cur_e.velocity>0.){ */
/* 			f_b = des_Cur_e.dmp[V] * Cur_e.velocity - mmm * Grav * sin(Cur_e.tilt[beta])- F_R * cos(TH_R); */
			
/* 	} */
/* 		if(Cur_e.tilt[beta]>0. && Cur_e.velocity<=0.){ */
/* 			f_b = -des_Cur_e.dmp[V] * Cur_e.velocity + mmm * Grav * sin(Cur_e.tilt[beta])- F_R * cos(TH_R); */
		   
/* 		} */
/* 		/\*�u���[�L���[�����g*\/ */
/* 		if(Cur_e.velocity>=0.) */
/* 		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega * H * H - mmm * Grav * sin(Cur_e.tilt[alpha]) * H_m + F_R * sin(TH_R) / H; */
/* 		else */
/* 			t_b = - des_Cur_e.dmp[TH]*Cur_e.omega * H * H + mmm * Grav * sin(Cur_e.tilt[alpha]) * H_m + F_R * sin(TH_R) / H; */
		
		
		/*�i�s�����u���[�L��*/
		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;
 		
		/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega * H * H 	+ F_R * sin(TH_R) / H ; 

		if(ticks%500==0){
			/* if(mindata_d[type]==step) printf("TYPE = STEP\n"); */
/* 			else if(mindata_d[type]==obstacle) printf("TYPE = OBSTACLE \n"); */
/* 			else printf("TYPE = UNKNOWN\n"); */
			printf("P_q= %f\tH=%f\n", P_q,H);
		}
		
		TorqCtrl();
		trig = 0;
		ticks++;
		return;
}



/* serqnx �ƃf�[�^�̂��Ƃ������@�@ �@�@�@�@�@�@�@�@�@�@�@�@�@�@*/
void get_sick_data(void)
{
	static unsigned long ticks=0;
	double CurData[4] = {0.};
	
	CurData[X] = Cur_e.pos[X];
	CurData[Y] = Cur_e.pos[Y];
	CurData[TH] = Cur_e.pos[TH];
	
//	memcpy(CurData, Cur_e.pos, sizeof(Cur_e.pos));	

	recv(new_fd, mindata_d, sizeof(mindata_d),0);
	send(new_fd, CurData, sizeof(CurData),0);

	if(ticks%20==0)
	printf("mindata=%f, mindata[ang]=%f, X=%f, Y=%f, TH=%f\n", mindata_d[dis], mindata_d[ang], CurData[X],CurData[Y],CurData[TH]);
	
/* 	mindata_d[dis] = (double)mindata[dis]*0.001; /\* from mm to m *\/ */
/* 	mindata_d[ang] = (double)mindata[ang]*1.0/2.0; */
/* 	mindata_d[ang] = deg2rad(mindata_d[ang]);/\* from deg to rad *\/ */
	ticks++;
}

/* getObstacleAbsoluteCoordinate                                  */
/* �r�h�b�j���瓾��ꂽ��Q���̋ɍ��W���΍��W�֕ϊ�����@�@�@   */
void getObstacleAbsoluteCoordinate(void)
{
	double minang;
	double cTH, sTH;
	double c_min, s_min;
	
	minang = mindata_d[ang];
	
	c_min = cos(minang);
	s_min = sin(minang);
	
	cTH = cos(Cur_e.pos[TH]);
	sTH = sin(Cur_e.pos[TH]);

	if(H<=0.531) {
		Cross_p[X]=Cur_e.pos[X]+0.531*cTH+mindata_d[dis]*c_min*cTH-mindata_d[dis]*s_min*sTH;
		Cross_p[Y]=Cur_e.pos[Y]+0.531*sTH+mindata_d[dis]*c_min*sTH+mindata_d[dis]*s_min*cTH;
	}
	else{
	Cross_p[X]=Cur_e.pos[X]+(H-0.531)*cTH+mindata_d[dis]*c_min*cTH-mindata_d[dis]*s_min*sTH;
	Cross_p[Y]=Cur_e.pos[Y]+(H-0.531)*sTH+mindata_d[dis]*c_min*sTH+mindata_d[dis]*s_min*cTH;
	}
}

/* SlopeCtrl                                                       */
/* command : slope(double dam[0], double dam[1])                   */
/* �@�@*�����m�[�g�T�@�Q�Ɓ@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ */
void
SlopeCtrl(struct params *param, int trig)
{
	static unsigned long ticks = 0;
	double mmm = 40.;
	
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH]= param->deltaDamper[TH];
		printf("SlopeCtrl Start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

	ADCtrl();
	
	/*�i�s�����u���[�L��*/

	if(Cur_e.tilt[beta]<=0.&& Cur_e.velocity>=0.){
		f_b = des_Cur_e.dmp[V] * Cur_e.velocity +mmm * Grav * sin(-Cur_e.tilt[beta]);
		if(ticks%500==0) printf("1\n");
	}
	if(Cur_e.tilt[beta]<=0 && Cur_e.velocity<0.){
		f_b = -des_Cur_e.dmp[V] * Cur_e.velocity + mmm * Grav * sin(-Cur_e.tilt[beta]); 
		if(ticks%500==0) printf("2\n");
	}
	if(Cur_e.tilt[beta]>0. && Cur_e.velocity>0.){
		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - mmm * Grav * sin(Cur_e.tilt[beta]);
		if(ticks%500==0) printf("3\n");
	}
	if(Cur_e.tilt[beta]>0. && Cur_e.velocity<=0.){
		f_b = -des_Cur_e.dmp[V] * Cur_e.velocity + mmm * Grav * sin(Cur_e.tilt[beta]);
		if(ticks%500==0) printf("4\n");
	}
	/*�u���[�L���[�����g*/
	if(Cur_e.velocity>=0.)
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega - mmm * Grav * sin(Cur_e.tilt[alpha]) * H_m;
	else
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega + mmm * Grav * sin(Cur_e.tilt[alpha]) * H_m;

	if(ticks%500==0)
		printf("alpha =%f, beta = %f, f_b = %f, t_b = %f\n", Cur_e.tilt[alpha] , Cur_e.tilt[beta], f_b, t_b);
	
	TorqCtrl();
	trig = 0;
	ticks++;
	return;
}

/* ADCtrl                                                           */
/* Read the data of tilt angle                                    */
void
ADCtrl(void)
{
	static unsigned long ticks = 0;
  
	float *adx, *ady;

	adx = adRead(ad, ch0);
	ady = adRead(ad, ch2);

	Cur_e.tilt[beta]  = volt2angle(ad->buffer[0]);
	Cur_e.tilt[alpha] = volt2angle(ad->buffer[2]);

	/* offset */
	Cur_e.tilt[beta]  = Cur_e.tilt[beta] - 0.014228;
	Cur_e.tilt[alpha] = Cur_e.tilt[alpha] - 0.037023;

	ticks++;
}

/* AvoidCtrl5                                                       */
/* command : avo5(double dam[0], double dam[1])                     */
/*  �@�������m�[�g5�@�Q�� �@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ */
void
AvoidCtrl5(struct params *param, int trig)
{
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("AvoidCtrl5 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
		printf("rou0 = %f, eta = %f\n", rou0, eta );
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

    /* 	Nearest Point */
   
	if( Cur_e.pos[Y] >= 0.&& Cur_e.pos[Y] >= Cur_e.pos[X]-1.9){
		/***<1>***/
		Cross_p[X] = Cur_e_c.pos[X];
		Cross_p[Y] = 0.9;
	}else if( Cur_e.pos[X] >1.9){
		/***<2>***/
		Cross_p[X] = 2.8;
		Cross_p[Y] = Cur_e_c.pos[Y];
	}else if( Cur_e.pos[Y] < 0. && Cur_e.pos[X] < 0.9 ){
		/***<3>***/
		Cross_p[X] = Cur_e_c.pos[X];
	    Cross_p[Y] = -0.9;
	}else if( Cur_e.pos[Y] < 0. && Cur_e.pos[Y] > -0.9 && Cur_e.pos[X] >= 0.9 && Cur_e.pos[X]<=1.9){
        /***<4>***/
		Cross_p[X] = 0.9;
		Cross_p[Y] = -0.9;
	}else if( Cur_e.pos[Y] <= -0.9 < Cur_e.pos[X]<=1.9){
		/***<5>***/
		Cross_p[X] = 0.9;
		Cross_p[Y] = Cur_e_c.pos[Y];
	}
	
	getNearestDistance();
	
	if(P_q<=rou0) RepulsiveForce(); 
	else F_R = 0.0;
	
	/*�i�s�����u���[�L��*/
    /*f_b = des_Cur_e.dmp[V] * Cur_e.velocity +(-F_R * cos(TH_R)) ;*/
	f_b = des_Cur_e.dmp[V] * Cur_e.velocity -F_R * cos(TH_R) ;
	/*��]�p�u���[�L��*/
	/*t_b = - (des_Cur_e.dmp[TH]*Cur_e.omega 	+( - F_R * sin(TH_R) / H )); */
    t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ;
	
	TorqCtrl();
	return;	
}


/* AvoidCtrl4                                                       */
/* command : avo4(double dam[0], double dam[1])                     */
/*  �@�������m�[�g5�@�Q�� �@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ */

void
AvoidCtrl4(struct params *param, int trig)
{
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("AvoidCtrl4 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
		printf("rou0 = %f, eta = %f\n", rou0, eta );
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

    /* 	Nearest Point */

	if( Cur_e.pos[Y] >= 0.){
		Cross_p[X] = Cur_e_c.pos[X];
		Cross_p[Y] = 1.0;
	}else if( Cur_e.pos[Y] < 0. && Cur_e.pos[X] < 1. ){
		Cross_p[X] = Cur_e_c.pos[X];
		Cross_p[Y] = -1.0;
	}else if( Cur_e.pos[Y] < 0. && Cur_e.pos[Y] > -1. && Cur_e.pos[X] >= 1.){
		Cross_p[X] = 1.;
		Cross_p[Y] = -1.;
	}else if( Cur_e.pos[Y] <= -1.){
		Cross_p[X] = 1.;
		Cross_p[Y] = Cur_e_c.pos[Y];
	}
	
	getNearestDistance();
	
	if(P_q<=rou0) RepulsiveForce(); 
	else F_R = 0.0;
	
	/*�i�s�����u���[�L��*/
 		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;

	/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 
	
	TorqCtrl();
	return;	
}

/* AvoidCtrl3                                                       */
/* command : avo3(double dam[0], double dam[1])                     */
/* 	y = 1.0 & y = -1.0 ��ǂƂ�����Q��������[�h                   */
/*  �@�������m�[�g�S�@�Q�� �@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ */

void
AvoidCtrl3(struct params *param, int trig)
{
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("AvoidCtrl3 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
		printf("rou0 = %f, eta = %f\n", rou0, eta );
	}
	
	getCurrentStatus();
	getCurrentControlPosition();

/* 	Nearest Point */
	
	if(Cur_e.pos[Y]<= 0.){
		Cross_p[X] = Cur_e_c.pos[X];
		Cross_p[Y] = -1.0;

	}
    else{
		Cross_p[X] = Cur_e_c.pos[X];
		Cross_p[Y] = 1.0;
	}

	getNearestDistance();
	
	if(P_q<=rou0) RepulsiveForce(); 
	else F_R = 0.0;
	
	/*�i�s�����u���[�L��*/
 		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;

	/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 
	
	TorqCtrl();
	return;	
}

/* AvoidCtrl2                                                       */
/*   command: avo2(double dam[0], double dam[1])                    */
/* �΂ߒ�������Q���Ƃ�����Q��������[�h �@�@�@                    */
/* �������m�[�g�S�@�Q��                                             */
/*                                                                  */
void
AvoidCtrl2(struct params *param, int trig)
{
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("AvoidCtrl2 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}

	getCurrentStatus();
	getCurrentControlPosition();
	
    /*the Nearest Point*/
	Cross_p[X] = ( 1.5 + Cur_e_c.pos[X] - Cur_e_c.pos[Y] ) / 2. ;
	Cross_p[Y] = ( 1.5 - Cur_e_c.pos[X] + Cur_e_c.pos[Y] ) / 2. ;
	
	getNearestDistance();

	if(P_q<=rou0) RepulsiveForce(); 
	else F_R = 0.0;
	
	/*�i�s�����u���[�L��*/
  		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;
		
	/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 
	
	TorqCtrl();

}

/* AvoidCtrl1                                                      */
/*   command: avo1(double dam[0], double[1]                        */
/* ���_(2.0, 1.5)����Q���Ƃ�����Q��������[�h                    */
/* �������m�[�g�S�@�Q�Ɓ@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@*/
/*                                                                 */
void
AvoidCtrl1(struct params *param, int trig)
{

	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("AvoidCtrl1 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
		Cross_p[X] = 2.0;
		Cross_p[Y] = 1.5;
	}
	
	getCurrentStatus();
	getCurrentControlPosition();
	getNearestDistance();

	if(P_q<=rou0) RepulsiveForce(); 
	else F_R = 0.0;
	
	/*�i�s�����u���[�L��*/
  
		f_b = des_Cur_e.dmp[V] * Cur_e.velocity - F_R * cos(TH_R) ;

	/*��]�p�u���[�L��*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 	+ F_R * sin(TH_R) / H ; 
	
	TorqCtrl();
	return;
}

/* RepulsiveForce                                                  */
/* ��΍��W�˗͐����C���˗́C���˗͕����@���v�Z                    */
void
RepulsiveForce(void)
{
//	double P_qqq;
	double F_rr;
	
/*	P_qqq = P_q*P_q*P_q;*/          /*�@P_q�̂R��@*/

	/*�@Khatib�̕��@�@*/
//	P_qqq = P_q*P_q*P_q;          /*�@P_q�̂R��@*/
//	F_r[X] = eta*(1/P_q - 1/rou0)*(Cur_e_c.pos[X]-Cross_p[X])/P_qqq;
//	F_r[Y] = eta*(1/P_q - 1/rou0)*(Cur_e_c.pos[Y]-Cross_p[Y])/P_qqq;

	/*  cos�֐���p�����˗�  */
	F_r[X] = mindata_d[type]*eta*PI*(Cur_e_c.pos[X]-Cross_p[X])*sin(PI*P_q/rou0)/(rou0*P_q);
	F_r[Y] = mindata_d[type]*eta*PI*(Cur_e_c.pos[Y]-Cross_p[Y])*sin(PI*P_q/rou0)/(rou0*P_q);

//	F_r[X] = eta*PI*(Cur_e_c.pos[X]-Cross_p[X])*sin(PI*P_q/rou0)/(rou0*P_q);
//	F_r[Y] = eta*PI*(Cur_e_c.pos[Y]-Cross_p[Y])*sin(PI*P_q/rou0)/(rou0*P_q);
	
	/*  exp�֐���p�����˗�  */
//	P_qqq  = rou0 - P_q;
//	F_r[X] = 2*eta*(P_qqq)*(Cur_e_c.pos[X]-Cross_p[X])*exp(-P_qqq*P_qqq);
//	F_r[Y] = 2*eta*(P_qqq)*(Cur_e_c.pos[Y]-Cross_p[Y])*exp(-P_qqq*P_qqq);
	
	/*  cos�֐���exp�֐����d�ˍ��킹���˗�*/

	/* P_qqq = rou0 - P_q; */
/* 	F_r[X] = eta*PI*(Cur_e_c.pos[X]-Cross_p[X])*sin(PI*P_q/rou0)/(rou0*P_q) */
/* 		        +2*eta*(P_qqq)*(Cur_e_c.pos[X]-Cross_p[X])*exp(-P_qqq*P_qqq); */
/* 	F_r[Y] = eta*PI*(Cur_e_c.pos[Y]-Cross_p[Y])*sin(PI*P_q/rou0)/(rou0*P_q) */
/* 	        +2*eta2*(P_qqq)*(Cur_e_c.pos[Y]-Cross_p[Y])*exp(-P_qqq*P_qqq); */

	
	F_R    = F_r[X]*F_r[X]+F_r[Y]*F_r[Y];
	F_R    = sqrt(F_R);
	
	F_rr= atan2(F_r[Y], F_r[X]);

	F_r[TH] = F_rr;
	TH_R   = -Cur_e.pos[TH]+F_r[TH];

	return ;
}

/* getNearestDistance                                               */
/* Walker�����Q���ւ̍ŋߐڋ��� P_q �����߂�                      */
void
getNearestDistance(void)
{
	P_q=(Cur_e_c.pos[X]-Cross_p[X])*(Cur_e_c.pos[X]-Cross_p[X])
		+(Cur_e_c.pos[Y]-Cross_p[Y])*(Cur_e_c.pos[Y]-Cross_p[Y]);
	P_q=sqrt(P_q);
	//TH_R=0;
	return;
}

/*�i�ڕW���C�W���j�[�i���ۂ̖��C�W���j���ڕW�ɂ��Ă���@�@�@�@�@�@�@*/
/* ���[���Ƃ����{�ł���Ă��܂����@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@*/
void
ModelCtrlOnly(struct params *param, int trig)
{

  if(trig){
    des_Cur_e.dmp[V] = param->deltaDamper[V];
	des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    printf("ModelCtrlOnly start!\n");
    printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  
  if( Cur_e.velocity > 0 ){
    /*�i�s�����u���[�L��*/
        f_b = des_Cur_e.dmp[V]*Cur_e.velocity ;
    /*��]�����u���[�L��*/
        /* t_b = - des_Cur_e.dmp[TH]*Cur_e.omega ; */          /*������o�[�W����*/
		t_b = - des_Cur_e.dmp[TH]*Cur_e.omega*H*H ;            /*  �L���X�^�[���� */
  } 
  else{
        f_b = -des_Cur_e.dmp[V]*Cur_e.velocity ;
        /* t_b = + des_Cur_e.dmp[TH]*Cur_e.omega ; */          /*������o�[�W����*/
		t_b = + des_Cur_e.dmp[TH]*Cur_e.omega*H*H ;            /* �L���X�^�[���� */
  }
  
  TorqCtrl();
  return;
  
}

/*��₱�����ł����C����͖ڕW�̖��CD_d�𒼐ڎw�肵�Ă���^�������ϐ���v���O����*/
/*�A���A���i�����̂݁D��]�Ɋւ��Ă͍����R�}���h���͂��Ă�������                   */
void
ModelCtrlOnly2(struct params *param, int trig)
{
	static double Dam = 10.; /*���ۂ̐i�s�����Ɋւ��門�C�W�������*/
	double dam;
	
	if(trig){
		des_Cur_e.dmp[V] = param->deltaDamper[V];
		des_Cur_e.dmp[TH] = param->deltaDamper[TH];
		printf("ModelCtrlOnly2 start!\n");
		printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
	}
	
	getCurrentStatus();
	
	dam = des_Cur_e.dmp[V] - Dam;
	
	if( Cur_e.velocity > 0 ){
		/*�i�s�����u���[�L��*/
        f_b = dam*Cur_e.velocity ;
		/*��]�����u���[�L��*/
        t_b = - des_Cur_e.dmp[TH]*Cur_e.omega ;
	}
	else{
        f_b = -dam*Cur_e.velocity ;
        t_b = + des_Cur_e.dmp[TH]*Cur_e.omega ;
	}
   
  TorqCtrl();
  return;
}

void
ModelCtrl5(struct params *param, int trig)
{
  double sTH;
  
  if(trig){
    des_Cur_e.dmp[V] = param->deltaDamper[V];
    des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    printf("ModelCtrl5 start!\n");
    printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  
  /* tTH = tan(Cur_e.pos[TH]);
     cTH = cos(Cur_e.pos[TH]);*/
  sTH = sin(Cur_e.pos[TH]);

  if(Cur_e.pos[X]<=0.5) Mod5_L1();
  if(Cur_e.pos[X]>  0.5 && Cur_e.pos[X]<= 1.6 && Cur_e.pos[Y]>-1.0) Mod5_L2();
  if(Cur_e.pos[Y]<=-1.0 && Cur_e.pos[Y]> -1.5 && Cur_e.pos[X]<=2.5) Mod5_L3();
  if(Cur_e.pos[Y]<=-1.5 && Cur_e.pos[Y]>=-2.5 && Cur_e.pos[X]< 2.5) Mod5_L4();
  if(Cur_e.pos[X]>= 2.5 && Cur_e.pos[Y]< -2.3 && Cur_e.pos[Y]>-2.7) Mod5_L5(); 

  if( Cur_e.velocity > 0 ){
    /*�i�s�����u���[�L��*/
    if( er[X] > 0.)
      f_b = des_Cur_e.dmp[V]*Cur_e.velocity + K_v * ( 1.- exp( - er[X] * er[X]))*sTH ;
    else
      f_b = des_Cur_e.dmp[V]*Cur_e.velocity - K_v * ( 1.- exp( - er[X]* er[X]))*sTH ;
    
    /*��]�����u���[�L��*/
    if( er[TH] >0.)
      t_b = - des_Cur_e.dmp[TH]*Cur_e.omega + K_th * ( 1.- exp( - er[TH]* er[TH]) );
    else
      t_b = - des_Cur_e.dmp[TH]*Cur_e.omega - K_th * ( 1.- exp( - er[TH]* er[TH]) ); 
      

  }
  else{
    if( er[X] > 0.)
      f_b = -des_Cur_e.dmp[V]*Cur_e.velocity - K_v * ( 1.- exp( - er[X] * er[X]))*sTH ;
    else
      f_b = -des_Cur_e.dmp[V]*Cur_e.velocity + K_v * ( 1.- exp( - er[X]* er[X]))*sTH ;

    if( er[TH] >0.)
      t_b = + des_Cur_e.dmp[TH]*Cur_e.omega - K_th * ( 1.- exp( - er[TH]* er[TH]) );
    else
      t_b = + des_Cur_e.dmp[TH]*Cur_e.omega + K_th * ( 1.- exp( - er[TH]* er[TH]) ); 
  }

  TorqCtrl();
  return;
  
}

void
Mod5_L1(void)
{
  er[X] = Cur_e.pos[Y];
  er[TH] = Cur_e.pos[TH];  
}

void
Mod5_L2(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.0 )*(Cur_e.pos[Y] + 1.0 ) 
     + (Cur_e.pos[X]-0.5)*(Cur_e.pos[X]-0.5));   
  er[X] = PR -1.0 ;
  des_th = -asin((Cur_e.pos[X]-0.5)/PR);
  er[TH] = Cur_e.pos[TH] - des_th;
}

void
Mod5_L3(void)
{
  er[X] = Cur_e.pos[X]-1.5;
  er[TH] = Cur_e.pos[TH]+PI/2.;
}

void
Mod5_L4(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.5)*(Cur_e.pos[Y] + 1.5)
           + (Cur_e.pos[X] - 2.5)*(Cur_e.pos[X] - 2.5));
  er[X] = -(PR - 1.0);
  des_th =  asin((Cur_e.pos[X]-2.5)/PR);
  er[TH] = Cur_e.pos[TH] - des_th;
}

void
Mod5_L5(void)
{
  er[X] = Cur_e.pos[Y] + 2.5;
  er[TH] = Cur_e.pos[TH];
}


void
ModelCtrl4(struct params *param, int trig)
{
  double tTH, cTH , sTH;
  
  if(trig){
    des_Cur_e.dmp[V] = param->deltaDamper[V];
    des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    printf("ModelCtrl4 start!\n");
    printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  
  tTH = tan(Cur_e.pos[TH]);
  cTH = cos(Cur_e.pos[TH]);
  sTH = sin(Cur_e.pos[TH]);

  /*�i�s�����u���[�L��*/
  if( Cur_e.pos[Y]>0.)
	  f_b = des_Cur_e.dmp[V]*Cur_e.velocity + K_v* ( 1.- exp( - Cur_e.pos[Y]*Cur_e.pos[Y]))*sTH ;
  else
	  f_b = des_Cur_e.dmp[V]*Cur_e.velocity - K_v* ( 1.- exp( - Cur_e.pos[Y]*Cur_e.pos[Y]))*sTH ;
  /*��]�p�u���[�L��*/
   if(Cur_e.pos[TH]>0.)
	   t_b = - des_Cur_e.dmp[TH]*Cur_e.omega + K_th* ( 1.- exp( - Cur_e.pos[TH]*Cur_e.pos[TH])); 
   else
	   t_b = - des_Cur_e.dmp[TH]*Cur_e.omega - K_th* ( 1.- exp( - Cur_e.pos[TH]*Cur_e.pos[TH])); 
     
  TorqCtrl();
  return;
  
}

void
ModelCtrl3a(struct params *param, int trig)
{
  double cTH , sTH;
  static unsigned long ticks = 0;

  if(trig){
    des_Cur_e.dmp[V] = param->deltaDamper[V];
    des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    printf("ModelCtrl3a start!\n");
    printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  getCurrentControlPosition();

  /*
  cTH = cos(Cur_e.pos[TH]);
  sTH = sin(Cur_e.pos[TH]);
  */

  cTH = cos(des_th);
  sTH = sin(des_th);

  if(Cur_e_c.pos[X]<= 0.5) Mod3a_L1();
  if(Cur_e_c.pos[X]>  0.5 && Cur_e_c.pos[X]<= 1.6 && Cur_e_c.pos[Y]>-1.0 ) Mod3a_L2();
  if(Cur_e_c.pos[Y]<=-1.0 && Cur_e_c.pos[Y]> -1.5 && Cur_e_c.pos[X]<=2.5 ) Mod3a_L3();
  if(Cur_e_c.pos[Y]<=-1.5 && Cur_e_c.pos[Y]>=-2.5 && Cur_e_c.pos[X]< 2.5 ) Mod3a_L4();
  if(Cur_e_c.pos[X]>=2.5) Mod3a_L5(); 

  /*�i�s�����u���[�L��*/
    if( PQ > 0.)
      f_b = des_Cur_e.dmp[V] * Cur_e.velocity + K_v * ( 1.- exp( - PQ * PQ ))*sTH ;
    else
      f_b = des_Cur_e.dmp[V] * Cur_e.velocity - K_v * ( 1.- exp( - PQ * PQ ))*sTH ;
 
  /*��]�p�u���[�L��*/
 
    if( PQ > 0.)
      t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 
		     - K_th * ( 1.- exp( - PQ * PQ ) ) * cTH / H ; 
    else
      t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 
		     + K_th * ( 1.- exp( - PQ * PQ ) ) * cTH / H ; 

	if(ticks%500==0)
		printf("Cur_e_c.pos[X]=%f, Cur_e_c.pos[Y]=%f, f_b = %f, t_b = %f, PQ = %f, des_th = %f\n", Cur_e_c.pos[X], Cur_e_c.pos[Y], f_b, t_b, PQ, des_th);

  TorqCtrl();

  ticks++;
  return;
  
}

void
Mod3a_L1(void)
{
  PQ = Cur_e_c.pos[Y];
  des_th = Cur_e.pos[TH];
  return;
}

void
Mod3a_L2(void)
{
	
  PR = sqrt((Cur_e_c.pos[Y] + 1.0 )*(Cur_e_c.pos[Y] + 1.0 ) 
     + (Cur_e_c.pos[X]-0.5)*(Cur_e_c.pos[X]-0.5));   

  PQ = PR -1.0 ;

  des_th = -asin((Cur_e_c.pos[X]-0.5)/PR);
  des_th = +Cur_e_c.pos[TH] - des_th;

  return;
}
void
Mod3a_L3(void)
{
  PQ = Cur_e_c.pos[X]-1.5;
  des_th = Cur_e_c.pos[TH]+(PI/2);
  return;
}

void
Mod3a_L4(void)
{
  PR = sqrt((Cur_e_c.pos[Y] + 1.5)*(Cur_e_c.pos[Y] + 1.5)
           + (Cur_e_c.pos[X] - 2.5)*(Cur_e_c.pos[X] - 2.5));
  PQ = -(PR - 1.0);
  des_th = asin((Cur_e_c.pos[X]-2.5)/PR);
  des_th = Cur_e.pos[TH] -  des_th;

  /* des_th =  asin((Cur_e_c.pos[X]-2.5)/PR);
  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;*/
  return;
}

void
Mod3a_L5(void)
{
  PQ = Cur_e_c.pos[Y]+2.5;
  des_th = Cur_e.pos[TH];
  return;
}


void
ModelCtrl3(struct params *param, int trig)
{
  double tTH, cTH , sTH;
  
  if(trig){
    des_Cur_e.dmp[V] = param->deltaDamper[V];
    des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    printf("ModelCtrl3 start! jayo\n");
    printf("D = %f, D_th = %f\n", des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  
  tTH = tan(Cur_e.pos[TH]);
  cTH = cos(Cur_e.pos[TH]);
  sTH = sin(Cur_e.pos[TH]);

  /*�i�s�����u���[�L��*/

  if( Cur_e.pos[Y]>0.)
  f_b = des_Cur_e.dmp[V]*Cur_e.velocity 
    + 500.* ( 1.- exp( - (Cur_e.pos[Y]+r_off*sTH)*(Cur_e.pos[Y]+r_off*sTH)))*sTH ;
  else
  f_b = des_Cur_e.dmp[V]*Cur_e.velocity 
    - 500.* ( 1.- exp( - (Cur_e.pos[Y]+r_off*sTH)*(Cur_e.pos[Y]+r_off*sTH)))*sTH ;

  /*��]�p�u���[�L��*/
 
  if( Cur_e.pos[Y] > 0.)
    t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 
      + 500.* ( 1.- exp( - (Cur_e.pos[Y]+r_off*sTH)*(Cur_e.pos[Y]+r_off*sTH)) ) * cTH / r_off; 
  else
    t_b = - des_Cur_e.dmp[TH]*Cur_e.omega 
      - 500.* ( 1.- exp( - (Cur_e.pos[Y]+r_off*sTH)*(Cur_e.pos[Y]+r_off*sTH)) ) * cTH / r_off; 
   
  TorqCtrl();
  return; 
}

void
ModelCtrl2(struct params *param, int trig)
{
  double tTH, sTH;

  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    K_eta = K_eta_d/des_Cur_e.velocity;
    des_Cur_e.dmp[V] = param->deltaDamper[V];
    des_Cur_e.dmp[TH] = param->deltaDamper[TH];
    des_Cur_e.mass[M] = param->deltaMass[M];
    des_Cur_e.mass[J] = param->deltaMass[J];
    printf("ModelCtrl2 start!\n");
    printf("M = %f, J = %f, D = %f, D_th = %f\n", des_Cur_e.mass[M], 
	   des_Cur_e.mass[J], des_Cur_e.dmp[V], des_Cur_e.dmp[TH]);
   }
  
  getCurrentStatus();
  TraCtrl5_alpha();

  tTH = tan(Cur_e.pos[TH]);
  sTH = sin(Cur_e.pos[TH]);
    
  /*�i�s�����u���[�L��*/
 
  if( Cur_e.pos[TH] >= (PI/2.-0.001) && Cur_e.pos[TH] <= (PI/2.+0.001))
    f_b = des_Cur_e.mass[M]* k_f_b*sin(Cur_e.pos[TH]+PI/2.)
	   *(-des_Cur_e.velocity+Cur_e.velocity)
	   +(des_Cur_e.mass[M]*Cur_e.omega*tan(Cur_e.pos[TH]+PI/2.)
	     +des_Cur_e.dmp[V])*Cur_e.velocity*sTH;
  else
    f_b = des_Cur_e.mass[M]*k_f_b*(-des_Cur_e.velocity+Cur_e.velocity)
      +(des_Cur_e.mass[M]*Cur_e.omega*tTH+des_Cur_e.dmp[V])*Cur_e.velocity ;
  
 /*��]�p�u���[�L��*/
  t_b = -(des_Cur_e.mass[J]*k_t_b*(des_Cur_e.omega-Cur_e.omega)+des_Cur_e.dmp[TH]*Cur_e.omega);

  TorqCtrl();
  return;
  
}

void
TorqCtrl(void)
{ 
  double torq[2], volt[2];

 /*�u���[�L�g���N�̌���*/
  torq[r] = MC_r*f_b/2. - MC_r*t_b/MC_T;
  torq[l] = MC_r*f_b/2. + MC_r*t_b/MC_T;
    
  /* �w�ߓd�� */
 /*   volt[r] = torq[r]/Torqconst_0;  */
/*    volt[l] = torq[l]/Torqconst_1;  */

  
/*   if(ticks%500==0) */
/*     printf("TYPE = %f, f_b=%f, t_b=%f, torq[r]=%f, torq[r] = %f, Cross_p[X] = %f, Cross_p[Y] = %f\n", */
/* 	   mindata_d[type], f_b, t_b, torq[r], torq[l], Cross_p[X], Cross_p[Y]); */
  
  /*left �␳*/
  
  /*�␳�P*/

  if(torq[r]>2.1393) volt[r] = (torq[r]+1.3393)/3.4786;
  else volt[r] = torq[r]/2.1393;
  
  if(torq[r]>2.1393) volt[l] = (torq[l]+1.3393)/3.4786;
  else volt[l] = torq[l]/2.1393;

  /*�␳2*/
/*   if(torq[r]>2.1393) volt[r] = (torq[r]+1.3393)/3.4786; */
/*   else if(torq[r]>1.06965 && torq[r]<=2.1393) volt[r] = sqrt((torq[r]-1.0695)/4.2786)+0.5; */
/*   else if(torq[r]>0.&& torq[r]<=1.06965) volt[r] = -sqrt((-torq[r]+1.0695)/4.2786)+0.5; */
/*   else volt[r] = torq[r]/2.13933; */
  
/*   if(torq[l]>2.1393) volt[l] = (torq[l]+1.3393)/3.4786; */
/*   else if(torq[l]>1.06965 && torq[l]<=2.1393) volt[l] = sqrt((torq[l]-1.0695)/4.2786)+0.5; */
/*   else if(torq[l]>0. && torq[l]<=1.06965) volt[l] = -sqrt((-torq[l]+1.0695)/4.2786)+0.5; */
/*   else volt[l] = torq[l]/2.13933; */
  /*volt[1] = volt[1]*4.1485/4.0717;*/
  
  if(volt[r] >  4.0)volt[r] =  4.0;
  if(volt[r] <  0.0)volt[r] =  0.0;

  if(volt[l] >  4.0)volt[l] =  4.0;
  if(volt[l] <  0.0)volt[l] =  0.0;
  
  output(volt, torq);

}

void
TraCtrl5_alpha(void)
{

 if(Cur_e.pos[X]<=0.5) Tra5_L1();
  else if(Cur_e.pos[X]>  0.5 && Cur_e.pos[X]<= 1.6 && Cur_e.pos[Y]>-1.0) Tra5_L2();
  else if(Cur_e.pos[Y]<=-1.0 && Cur_e.pos[Y]> -1.5 && Cur_e.pos[X]<=2.5) Tra5_L3();
  else if(Cur_e.pos[Y]<=-1.5 && Cur_e.pos[Y]>=-2.5 && Cur_e.pos[X]< 2.5 ) Tra5_L4();
  else Tra5_L5(); 
}

void
TraCtrl6(struct params *param, int trig)
{

  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    K_eta = K_eta_d/des_Cur_e.velocity;
  }

  getCurrentStatus();

  getCurrentControlPosition();

  if(Cur_e_c.pos[X]<=0.5) Tra6_L1();
  else if(Cur_e_c.pos[X]>  0.5 && Cur_e_c.pos[X]<= 1.6 && Cur_e_c.pos[Y]>-1.0) Tra6_L2();
  else if(Cur_e_c.pos[Y]<=-1.0 && Cur_e_c.pos[Y]> -1.5 && Cur_e_c.pos[X]<=2.5) Tra6_L3();
  else if(Cur_e_c.pos[Y]<=-1.5 && Cur_e_c.pos[Y]>=-2.5 && Cur_e_c.pos[X]< 2.5 ) Tra6_L4();
  else Tra6_L5(); 

  /*  printf("des_Cur_e.omega %f\n",des_Cur_e.omega);*/
  Pre();
  Inv();
  
  torqCtrl();
  
  return;

}
void
Tra6_L1(void)
{
  des_Cur_e.omega = Cur_e.omega - ( K_omega * Cur_e.omega + K_theta * Cur_e.pos[TH]
       			    + K_eta * Cur_e_c.pos[Y])*TICKS;

   return;
}

void
Tra6_L2(void)
{
  PR = sqrt((Cur_e_c.pos[Y] + 1.0 )*(Cur_e_c.pos[Y] + 1.0 ) 
     + (Cur_e_c.pos[X]-0.5)*(Cur_e_c.pos[X]-0.5));   

  PQ = PR -1.0 ;
  des_th = -asin((Cur_e_c.pos[X]-0.5)/PR);
  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;
   return;
}
void
Tra6_L3(void)
{
  des_Cur_e.omega = Cur_e.omega - 
    ( K_omega * Cur_e.omega + K_theta * (Cur_e_c.pos[TH]+(PI/2))
				    + K_eta * (Cur_e_c.pos[X]-1.5))*TICKS;
  return;
}

void
Tra6_L4(void)
{
  PR = sqrt((Cur_e_c.pos[Y] + 1.5)*(Cur_e_c.pos[Y] + 1.5)
           + (Cur_e_c.pos[X] - 2.5)*(Cur_e_c.pos[X] - 2.5));
  PQ = -(PR - 1.0);
  des_th =  asin((Cur_e_c.pos[X]-2.5)/PR);
  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;
  return;
}

void
Tra6_L5(void)
{
  des_Cur_e.omega = Cur_e.omega - ( K_omega * Cur_e.omega + K_theta *Cur_e.pos[TH]
				    + K_eta * (Cur_e_c.pos[Y]+2.5))*TICKS;
  return;
}

void
getCurrentControlPosition(void)
{
  Cur_e_c.pos[TH] = Cur_e.pos[TH];
  Cur_e_c.pos[X] = Cur_e.pos[X] + H*cos(Cur_e_c.pos[TH]);
  Cur_e_c.pos[Y] = Cur_e.pos[Y] + H*sin(Cur_e_c.pos[TH]);

  return;
}

void
TraCtrl5(struct params *param, int trig)
{

  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    K_eta = K_eta_d/des_Cur_e.velocity;
  }

  getCurrentStatus();

  if(Cur_e.pos[X]<=0.5) Tra5_L1();
  else if(Cur_e.pos[X]>  0.5 && Cur_e.pos[X]<= 1.6 && Cur_e.pos[Y]>-1.0) Tra5_L2();
  else if(Cur_e.pos[Y]<=-1.0 && Cur_e.pos[Y]> -1.5 && Cur_e.pos[X]<=2.5) Tra5_L3();
  else if(Cur_e.pos[Y]<=-1.5 && Cur_e.pos[Y]>=-2.5 && Cur_e.pos[X]< 2.5 ) Tra5_L4();
  else Tra5_L5(); 

  /*  printf("des_Cur_e.omega %f\n",des_Cur_e.omega);*/
  Pre();
  Inv();
  
  torqCtrl();
  
  return;

}
void
Tra5_L1(void)
{
  des_Cur_e.omega = Cur_e.omega - ( K_omega * Cur_e.omega + K_theta * Cur_e.pos[TH]
				    + K_eta * Cur_e.pos[Y])*TICKS;
  return;
}

void
Tra5_L2(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.0 )*(Cur_e.pos[Y] + 1.0 ) 
     + (Cur_e.pos[X]-0.5)*(Cur_e.pos[X]-0.5));   

  PQ = PR -1.0 ;
  des_th = -asin((Cur_e.pos[X]-0.5)/PR);
  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;
  return;
}
void
Tra5_L3(void)
{
  des_Cur_e.omega = Cur_e.omega - 
    ( K_omega * Cur_e.omega + K_theta * (Cur_e.pos[TH]+(PI/2))
				    + K_eta * (Cur_e.pos[X]-1.5))*TICKS;
  return;
}

void
Tra5_L4(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.5)*(Cur_e.pos[Y] + 1.5)
           + (Cur_e.pos[X] - 2.5)*(Cur_e.pos[X] - 2.5));
  PQ = -(PR - 1.0);
  des_th =  asin((Cur_e.pos[X]-2.5)/PR);
  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;
  return;
}
void
Tra5_L5(void)
{
  des_Cur_e.omega = Cur_e.omega - ( K_omega * Cur_e.omega + K_theta * Cur_e.pos[TH]
				    + K_eta * (Cur_e.pos[Y]+2.5))*TICKS;
  return;
}
void
TraCtrl4(struct params *param, int trig)
{
  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    K_eta = K_eta_d/des_Cur_e.velocity;
   
  }

  getCurrentStatus();

  if(Cur_e.pos[Y]<=-1.19)
    tra4_L2();
  else tra4_L1();

  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;

  /*  printf("des_Cur_e.omega %f\n",des_Cur_e.omega);*/
  Pre();
  Inv();
  
  torqCtrl();
  
  return;

}

void
tra4_L1(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.2 )*(Cur_e.pos[Y] + 1.2 ) 
     + Cur_e.pos[X]*Cur_e.pos[X]);   

  PQ = PR -1.2 ;
  des_th = -asin(Cur_e.pos[X]/PR);
 
  return;
}

void
tra4_L2(void)
{
  PR = sqrt((Cur_e.pos[Y] + 1.2)*(Cur_e.pos[Y] + 1.2)
           + (Cur_e.pos[X] - 2.4)*(Cur_e.pos[X] - 2.4));
  PQ = -(PR - 1.2);
  des_th =  asin((Cur_e.pos[X]-2.4)/PR);
  /* printf("PR %f, PQ %f\n", PR, PQ, des_th);*/
  return;
}

void
TraCtrl3(struct params *param, int trig)
{
  double R;

  if(trig){  
    des_Cur_e.velocity = param->desVelocity;
    des_Cur_e.Rad = param->desR;

    K_eta = K_eta_d/des_Cur_e.velocity;

    R = sqrt(des_Cur_e.Rad * des_Cur_e.Rad);

    printf("TraCtrl2 start ! des_Cur_e %f R=%f\n",des_Cur_e.velocity, des_Cur_e.Rad);
 
}
  getCurrentStatus();

PR = sqrt((Cur_e.pos[Y] - des_Cur_e.Rad )*(Cur_e.pos[Y] - des_Cur_e.Rad ) 
     + Cur_e.pos[X]*Cur_e.pos[X]);   

/*�@�o�H�ƌ��ݒn�Ƃ̍ŏ������@*/
  R = sqrt(des_Cur_e.Rad * des_Cur_e.Rad);

  PQ = - PR + R ;
	    
  des_th = asin(Cur_e.pos[X]/PR);

  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;

  printf("des_Cur_e.omega = %f,PQ %f, PR %f, R %f\n", des_Cur_e.omega, PQ, PR, R);

  Pre();
  Inv();
  
  torqCtrl();

  return;

}

void
TraCtrl2(struct params *param, int trig)
{
  double R;

  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    des_Cur_e.Rad = param->desR;

    K_eta = K_eta_d/des_Cur_e.velocity;

    R = sqrt(des_Cur_e.Rad * des_Cur_e.Rad);

    printf("TraCtrl2 start ! des_Cur_e %f R=%f\n",des_Cur_e.velocity, des_Cur_e.Rad);
  }

  getCurrentStatus();
 
  PR = sqrt((Cur_e.pos[Y] - des_Cur_e.Rad )*(Cur_e.pos[Y] - des_Cur_e.Rad ) 
     + Cur_e.pos[X]*Cur_e.pos[X]);   

 /*�@�o�H�ƌ��ݒn�Ƃ̍ŏ������@*/
  R = sqrt(des_Cur_e.Rad * des_Cur_e.Rad);

  PQ = PR - R ;
	    
  /*des_th = atan(Cur_e.pos[X]/(Cur_e.pos[Y]-R)); *//*�@�ڕW�p�x(�ڐ��p�x)�@*/
  
  des_th = -asin(Cur_e.pos[X]/PR);

  des_Cur_e.omega = Cur_e.omega -( K_omega * Cur_e.omega + 
                  K_theta * (Cur_e.pos[TH]-des_th) + K_eta * PQ)*TICKS;
  

  /*printf("des_Cur_e.omega=%f, des_th=%f PQ = %f\n", des_Cur_e.omega, des_th, PQ); 
   */
  Pre();
  Inv();

  torqCtrl();
  trig = 0;

  return;
}

void
TraCtrl(struct params *param, int trig)
{

  if(trig){
    des_Cur_e.velocity = param->desVelocity;
    
    K_eta = K_eta_d/des_Cur_e.velocity;
    
    printf("TraCtrl start ! des_Cur_e %f \n",des_Cur_e.velocity);
  }

  getCurrentStatus();
 
  des_Cur_e.omega = Cur_e.omega - ( K_omega * Cur_e.omega + K_theta * Cur_e.pos[TH]
				    + K_eta * Cur_e.pos[Y])*TICKS;

  Pre();
  Inv();
  torqCtrl();
  trig = 0;

  return;
}

void
DirCtrl(struct params *param, int trig)
{

  if(trig){
    des_Cur_e.velocity=param->desVelocity;
    des_Cur_e.omega = param->desOmega;
    Inv();  
    printf("DirCtrl des_Cur_e %f, %f\n", des_Cur_e.velocity, des_Cur_e.omega);
    printf("DirCtrl des_Cur_j %f, %f\n", des_Cur_j.vel[0], des_Cur_j.vel[1]);
}
 
  pre_des_Cur_j.vel[0]=des_Cur_j.vel[0];
  pre_des_Cur_j.vel[1]=des_Cur_j.vel[1];

  getCurrentStatus();
 
  torqCtrl();
  trig = 0;
  return;
}


void
Inv(void)
{
  des_Cur_j.vel[0]=des_Cur_e.velocity/MC_r + MC_T*des_Cur_e.omega/(2*MC_r);
  des_Cur_j.vel[1]=des_Cur_e.velocity/MC_r - MC_T*des_Cur_e.omega/(2*MC_r);
 
   return;
}

void
Pre(void)
{
  pre_des_Cur_j.vel[0]=des_Cur_j.vel[0];
  pre_des_Cur_j.vel[1]=des_Cur_j.vel[1];
 
}

void
VelCtrl(struct params *param, int trig)
{

   if(trig){
    des_Cur_j.vel[0]=param->desVel[0];
    des_Cur_j.vel[1]=param->desVel[1];  
    printf(" des %f, %f\n", des_Cur_j.vel[0], des_Cur_j.vel[1]);
  }
   pre_des_Cur_j.vel[0]=des_Cur_j.vel[0];
   pre_des_Cur_j.vel[1]=des_Cur_j.vel[1];
   /*  printf(" des %f, %f\n", des_Cur_j.vel[0], des_Cur_j.vel[1]);*/

  getCurrentStatus();

  torqCtrl();
  trig = 0;
  return;
}

void
AD_test(struct params *param, int trig)
{
	static unsigned long ticks = 0;
	float *ad1, *ad2;
	
	if(trig){
		printf(" ad_test mode start! \n");
	}
	ad1 = adRead(ad, ch0);
	ad2 = adRead(ad, ch2);	
	
	if(ticks % 500==0)
	printf("ch0 = %f, ch2 = %f\n", ad->buffer[0], ad->buffer[2]);
	
	torqCtrl();
	trig = 0;
	ticks++;
	return;
}


void
getCurrentStatus(void)
{
    int    i;
    long   *countbuff;
    static unsigned long ticks = 0;   

    /* wheel */
    for(i=0;i<2;i++){
        pre_Cur_j.pos[i] = Cur_j.pos[i];
	pre_Cur_j.vel[i] = Cur_j.vel[i];
    }

    pre_Cur_e.velocity=Cur_e.velocity;
    pre_Cur_e.omega = Cur_e.omega;

    countbuff = cntRead(cnt);

    /*�ԗ֊p�x[rad]*/
    Cur_j.pos[0] =  count2angle_R(*(countbuff));    
    Cur_j.pos[1] =  count2angle_L(*(countbuff+1));
   
    for(i=0;i<2;i++){                      
      /*�e�ԗ֊p���x�̌v�Z*/
      Cur_j.vel[i]=(Cur_j.pos[i]-pre_Cur_j.pos[i])/ TICKS;
      /*�e�ԗ֑��x�̌v�Z*/
      Cur_w.vel[i]= MC_r * Cur_j.vel[i];
        }
   
    /*���x�̑傫�����v�Z*/
    Cur_e.velocity = ( Cur_w.vel[0] + Cur_w.vel[1] ) / 2 ;
    Cur_e.omega = ( Cur_w.vel[0] - Cur_w.vel[1] ) / MC_T ;    
    ticks++;
    getCurrentPosition();

    /* ��\�_�̒������v�Z                         */
	/* ���x�̕ω��ɂ���āA��\�_�̋������ω����� */

	if(Cur_e.velocity >= 0){
		//	H = 1.5 * Cur_e.velocity * Cur_e.velocity + 0.2;
		H = 0.531;
		DH = 0.531;
	}
	else{
		H = -0.2;
		DH = -0.2;
	}


	
	
}

/*                                                                            */
/*    getCurrentPosition()                                                    */
/*       forward kinematics                                                   */

void 
getCurrentPosition(void)
{
    pre_Cur_e.pos[X]=Cur_e.pos[X];
    pre_Cur_e.pos[Y]=Cur_e.pos[Y];
    pre_Cur_e.pos[TH]=Cur_e.pos[TH];
   
  /*theta*/
  Cur_e.pos[TH] = ((Cur_e.omega + pre_Cur_e.omega)*TICKS)/2 + pre_Cur_e.pos[TH];

  /*X*/
  /*
  Cur_e.pos[X] = ((Cur_e.velocity*cos(Cur_e.pos[TH])+pre_Cur_e.velocity*cos(pre_Cur_e.pos[TH]))
   *TICKS)/2+pre_Cur_e.pos[X];
   */
  Cur_e.pos[X] = Cur_e.velocity*cos(Cur_e.pos[TH])*TICKS+pre_Cur_e.pos[X];

  /*Y*/
  Cur_e.pos[Y] = ((Cur_e.velocity*sin(Cur_e.pos[TH])+pre_Cur_e.velocity*sin(pre_Cur_e.pos[TH]))
                 *TICKS)/2+pre_Cur_e.pos[Y];
	
}
	
void
torqCtrl(void)
{
  double accel[2], volt[2], torq[2];
  double I[2]={0.};
  
  /*  printf("%f, %f \n", des_Cur_j.vel[0], des_Cur_j.vel[1]);*/
  if(torq[0]<0) {
         Ki_control[0] = 0.;
	 I[0] = 0.;
    }
  else   Ki_control[0] = Ki[0];

  if(torq[1]<0){
    Ki_control[1]=0.;
    I[1]=0.;
  }
  else  Ki_control[1]=Ki[1];

    I[0]+= des_Cur_j.vel[0]-Cur_j.vel[0];
    I[1]+= des_Cur_j.vel[1]-Cur_j.vel[1];
    
    /* �ڕW�p�����x */
    
    des_Cur_j.acc[0] = (des_Cur_j.vel[0]-pre_des_Cur_j.vel[0])/TICKS;
    des_Cur_j.acc[1] = (des_Cur_j.vel[1]-pre_des_Cur_j.vel[1])/TICKS;

    /* PI���� */
    accel[0] = des_Cur_j.acc[0]+Kp[0]*(des_Cur_j.vel[0]-Cur_j.vel[0])+Ki_control[0]*I[0];
    accel[1] = des_Cur_j.acc[1]+Kp[1]*(des_Cur_j.vel[1]-Cur_j.vel[1])+Ki_control[1]*I[1];

    /* Torque */
    torq[0] =-( Inertia[0]*accel[0] + Damper[0]*Cur_j.vel[0] );
    torq[1] =-( Inertia[1]*accel[1] + Damper[1]*Cur_j.vel[1] );   

    /* �w�ߓd�� */
    volt[0] = torq[0]/Torqconst_0;
    volt[1] = torq[1]/Torqconst_1;  

    /*left �␳*/

    /*    volt[1] = volt[1]*4.1485/4.0717;*/
    
    if(volt[0] >  4.0)volt[0] =  4.0;
    if(volt[0] <  0.0)volt[0] =  0.0;

    if(volt[1] >  4.0)volt[1] =  4.0;
    if(volt[1] <  0.0)volt[1] =  0.0;
  
  output(volt, torq);
}
/*                                                                            */
/*    Nop ( void )                                                            */
/*                                                                            */
/*    --- called ---                                                          */
/*      control()                                                             */
/*                                                                            */
void
Nop(void)
{
    zeroout(da);
    ctrlEndFlag = 1;
}

/*                                                                            */
/*    end_sick ( void )                                                       */
/*                                                                            */
/*    --- called ---                                                          */
/*      fin()                                                                 */
/*                                                                            */
void
end_sick(void)
{
	double CurData[4] = {0.};
	
	CurData[X] = Cur_e.pos[X];
	CurData[Y] = Cur_e.pos[Y];
	CurData[TH] = Cur_e.pos[TH];
	CurData[3] = 1.;

	recv(new_fd,sickdata,sizeof(sickdata),0);
	send(new_fd,CurData,sizeof(CurData),0);
	close(new_fd);
}

/*                                                                            */
/*    CurCtrl ( void )                                                       */
/*                                                                            */
/*    --- explanation ---                                                     */
/*      output()                                                              */
/*                                                                            */
void
CurCtrl(struct params *param)
{
    int i;
    double volt[2]={0.};
    double torq[2]={0.};

    volt[0]=param->desVol[0];
    volt[1]=param->desVol[1];

    torq[0]=volt[0]*Torqconst_0;
    torq[1]=volt[0]*Torqconst_1;

    for(i=0;i<2;i++){
        if(volt[i] >  4.0)        volt[i] =  4.0;
        if(volt[i] <  0.0)        volt[i] =  0.0;
        }
   
    output(volt, torq);

    return;
}

/*                                                                            */
/*    Output ( void )                                                         */
/*                                                                            */
/*    --- called ---                                                          */
/*      CurCtrl()                                                             */
/*                                                                            */
void
output(double volt[], double torq[])
{
    static double buff[17] = { 0. };
    static unsigned long tickss = 0;

    buff[0]  = volt[0];
    buff[1]  = volt[1];
    buff[2]  = Cur_e.pos[X];
    buff[3]  = Cur_e.pos[Y];
    buff[4]  = Cur_e.pos[TH];
    buff[5]  = Cur_e.velocity;
    buff[6]  = Cur_e.omega;
    buff[7]  = Cur_e_c.pos[X];
    buff[8]  = Cur_e_c.pos[Y];
    buff[9]  = f_b;
    buff[10] = t_b;
    buff[11] = torq[0];
    buff[12] = torq[1];
	buff[13] = TH_R;
    buff[14] = F_R;
/*     buff[15] = F_r[X]; */
/*     buff[16] = F_r[Y]; */
/* 	buff[14] = mindata_d[dis]; */
	/* buff[15] = mindata_d[ang]; */
/* 	buff[16] = mindata_d[type]; */
	
	buff[15] = H;
	buff[16] = P_q;

	
//	buff[15]=Cross_p[X];
//	buff[16]=Cross_p[Y];
	
    /*    printf("%f %f %f %f \n", des_Cur_e.velocity, des_Cur_e.omega, des_Cur_j.vel[0],
            des_Cur_j.vel[1]);
    */
    daOut(da, buff);
    ctrlEndFlag = 1;
	
    logstrig = 1;
    
    if(logstrig){
     	      if((tickss%(int)(logsTICKS*FREQ))==0){
		logcnt = logSave(buff);
		/*printf("%f, %f, %f, %f\n", buff[0], buff[1], buff[2]*180./PI, 
			buff[3]*180./PI);*/
		/*printf("%f, %f, %f, %f\n", buff[0], buff[1], buff[2], buff[3]);*/
	      }

    }                  
    tickss++;

    return;
}

void
voltage(double vol[])
{
  Motorp.desVol[0] = vol[0];
  Motorp.desVol[1] = vol[1];
  Motorp.mode=current;

}

void
velocities(double velo[])
{
  Motorp.desVel[0] = velo[0];
  Motorp.desVel[1] = velo[1];
  Motorp.mode = vel_control;
  Motorp.trig = TRUE;
}

void velwomega(double velo, double ome)
{
  Motorp.desVelocity = velo;
  Motorp.desOmega = ome;
  Motorp.mode = direct_control;
  Motorp.trig = TRUE;
}

void trace(double veloc)
{
  Motorp.desVelocity = veloc;
  Motorp.mode = trace_control;
  Motorp.trig = TRUE;
}

void trace2(double veloc, double radius)
{
  Motorp.desVelocity = veloc;
  Motorp.desR = radius;
  Motorp.mode = trace_control_2;
  Motorp.trig = TRUE;
}

void trace3(double velo, double radius)
{
  Motorp.desVelocity = velo;
  Motorp.desR = radius;
  Motorp.mode = trace_control_3;
  Motorp.trig = TRUE;
}

void trace4(double velo)
{
  Motorp.desVelocity = velo;
  Motorp.mode = trace_control_4;
  Motorp.trig = TRUE;
}

void trace5(double velo)
{
  Motorp.desVelocity = velo;
  Motorp.mode = trace_control_5;
  Motorp.trig = TRUE;
}

void trace6(double velo)
{
  Motorp.desVelocity = velo;
  Motorp.mode = trace_control_6;
  Motorp.trig = TRUE;
}

/* command */
void tra(double veloc)
{
  double velocity = 0.0;

  velocity=veloc;

  trace(velocity);
}

void tra2(double veloc, double rad)
{
  double velocity = 0.0;
  double radius = 0.0;

  velocity=veloc;
  radius = rad;

  trace2(velocity, radius);
}

void tra3(double velo, double rad)
{
  double velocity = 0.0;
  double radius = 0.0;

  velocity = velo;
  radius = rad;

  trace3(velocity, radius);
}

void tra4(double velo)
{
  double velocity = 0.0;

  velocity = velo;

  trace4(velocity);
}

void tra5(double velo)
{
  double velocity = 0.0;
  velocity = velo;
  trace5(velocity);
}

void tra6(double velo)
{
  double velocity = 0.0;
  velocity = velo;
  trace6(velocity);
}

void mod2(double velo, double dam0, double dam1, double mas, double ine)
{
  Motorp.desVelocity = velo;
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.deltaMass[M]    = mas;
  Motorp.deltaMass[J]    = ine;
  Motorp.mode = model_control_2;
  Motorp.trig = TRUE;
}

void mod3(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_3;
  Motorp.trig = TRUE;
}  

void modcas(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_cas ;
  Motorp.trig = TRUE;
}

void mod4(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_4;
  Motorp.trig = TRUE;
}  

void mod5(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_5;
  Motorp.trig = TRUE;
}  

void modonly(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_only;
  Motorp.trig = TRUE;
}

void modmod(double dam0, double dam1)
{
  Motorp.deltaDamper[V]  = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = model_control_only2;
  Motorp.trig = TRUE;
}

void avo1(double dam0, double dam1)
{
  Motorp.deltaDamper[V] = dam0;
  Motorp.deltaDamper[TH] = dam1;
  Motorp.mode = avoid_control_1;
  Motorp.trig = TRUE;
}

void avo2(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = avoid_control_2;
	Motorp.trig = TRUE;
}

void avo3(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = avoid_control_3;
	Motorp.trig = TRUE;
}

void avo4(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = avoid_control_4;
	Motorp.trig = TRUE;
}

void avo5(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = avoid_control_5;
	Motorp.trig =TRUE;
}

void slope(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = slope_control;
	Motorp.trig = TRUE;
}

void lms(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = lms_control;
	Motorp.trig = TRUE;
}

void demo(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = demo_control;
	Motorp.trig = TRUE;
}

void obs(double dam0, double dam1)
{
	Motorp.deltaDamper[V]  = dam0;
	Motorp.deltaDamper[TH] = dam1;
	Motorp.mode = obs_control;
	Motorp.trig = TRUE;
}

void velome(double velo, double rev_radius)
{ 
  double velocity = 0.0;
  double omega = 0.0;

  velocity = velo;
  omega = velocity / rev_radius;
  printf("direct control start\n");
  velwomega(velocity, omega);
}

void out(double rv,double lv){

  /*double rv, lv;*/

  double vol[2] = { 0.0, 0.0 };

  /*printf("rv = %f, lv =%f\n", rv, lv);*/

  vol[0] = rv;
  vol[1] = lv;

  voltage(vol);
}

void adtest(void){
	Motorp.mode = ad_test;
	Motorp.trig = TRUE;
}

void 
vel(double vel_r, double vel_l){

  double velo[2]={ 0.0, 0.0 };

  printf("velocity control start R = %f [rad/s] L = %f[rad/s]\n", vel_r , vel_l);

  velo[0] = vel_r;
  velo[1] = vel_l;
  velocities(velo);
}



void
logw(void)
{
    logsoff();
    logWrite();
}

void
logsoff(void)
{
    logstrig = OFF;
    printf("CP > Data off. \n");
}

/*                                                                            */
/*    fin ( void )                                                            */
/*                                                                            */
/*    --- explanation ---                                                     */
/*      fin.                                                                  */
/*         - Stop Timer                                                       */
/*         - Close Drivers                                                    */
/*         - Delete Tasks                                                     */
/*         - Delete Semaphore(s)                                              */
/*                                                                            */
void
fin(void)
{
    Motorp.mode = nop;
 
    zeroout(da);
 
    logw();
    ctrltrig = 0;
    
    timer_delete(timerid);
    logstrig = 0;
	
    daClose(da);
    cntClose(cnt);
	adClose(ad);
	end_sick();
	
    printf("fin\n");

    ctrlTaskID = 0;
    logsTaskID = 0;
    maintrig = 0;
    
}

int
main(void)
{
    
    maintrig = 1;
    
    cp(NULL, NULL);

//    while(maintrig){;}
    
    
    return(OK);

}












