typedef struct {
    int             si_signo;
    int             si_code;
    int             si_errno;
    union {
        int             __pad[7];
        struct {
            pid_t           __pid;
            union {
                struct {
                    uid_t           __uid;
                    union sigval    __value;
                } __kill;     /* si_code <= 0 SI_FROMUSER */
                struct {
                    _CSTD clock_t   __utime;
                    int             __status; /* CLD_EXITED status, else signo */
                    _CSTD clock_t   __stime;
                } __chld; /* si_signo=SIGCHLD si_code=CLD_* */
            } __pdata;
        } __proc;
        struct {
            int             __fltno;
            void            *__fltip;   
            void            *__addr;    
            int             __bdslot;
        } __fault;                /* si_signo=SIGSEGV,ILL,FPE,TRAP,BUS */
    } __data;
} siginfo_t;

#define si_pid      __data.__proc.__pid
#define si_value    __data.__proc.__pdata.__kill.__value
#define si_uid      __data.__proc.__pdata.__kill.__uid
#define si_status   __data.__proc.__pdata.__chld.__status
#define si_utime    __data.__proc.__pdata.__chld.__utime
#define si_stime    __data.__proc.__pdata.__chld.__stime
#define si_fltno    __data.__fault.__fltno
#define si_trapno   si_fltno
#define si_addr     __data.__fault.__addr
#define si_fltip    __data.__fault.__fltip
#define si_bdslot   __data.__fault.__bdslot