/* timer.c */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/siginfo.h>
#include <sys/neutrino.h>

#include "timer.h"

int debug = 0;
int chid;
timer_t timerid;
struct itimerspec timer;

void
setupTimer (void)
{
    struct sigevent event;
    int coid;
    char *funcname = "setupTimer";
    
    coid = ConnectAttach (0, 0, chid, 0, 0);
    
    if(coid == -1){
        fprintf (stderr, "%s: couldn't ConnectAttach!\n",funcname);
        perror (NULL);
        exit (EXIT_FAILURE);
    }
    
    if(debug) printf("Connect Attach done well\n");
    
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, CODE_TIMER, 0);
    
    if (timer_create (CLOCK_REALTIME, &event, &timerid) == -1){
        fprintf (stderr, "%s: can't timer_create,errno %d\n",funcname, errno);
        perror (NULL);
        exit (EXIT_FAILURE);
    }
    
    
    timer.it_value.tv_sec = 1;
    timer.it_value.tv_nsec = 0;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_nsec = 2000000;

}


/* Add to init() this code

    ThreadCtl(_NTO_TCTL_IO,0);
    
    if ((chid = ChannelCreate (0)) == -1){
        fprintf (stderr,"timer.c: couldn't create channel!\n");
        perror (NULL);
        exit (EXIT_FAILURE);
    }
    
    setupTimer();

*/


/* Add to start() this code

    timer_settime (timerid, 0, &timer, NULL);

*/


/* Replace ctrlTask() (or any task function) with this function

int
task-function-name (?)
{
    int rcvid;
    MessageT msg;

    for(;;){
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if(rcvid == 0){                                // Timer Pulse Received
            control(?);
        }
    }
    
    return (EXIT_SUCCESS);
}

*/
