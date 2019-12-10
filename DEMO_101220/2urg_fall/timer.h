/* timer.h */

#define MT_WAIT_DATA	2	// message from client
#define MT_SEND_DATA	3	// message from client

#define CODE_TIMER		1	// pulse from timer

#define MT_OK			0	// message to client
#define MT_TIMEDOUT		1	// message to client

#define MAX_CLIENT		16


typedef struct
{
    int messageType;
    int messageData;
} ClientMessageT;


typedef union
{
    ClientMessageT msg;
    struct _pulse pulse;
} MessageT;


struct
{
    int in_use;
    int rcvid;
    int timeout;
} clients [MAX_CLIENT];


extern int chid;
extern timer_t timerid;
extern struct itimerspec timer;

extern void setupTimer (void);
