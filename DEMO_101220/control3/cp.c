/* cp.c     : Command Processing Server                                       */
/*                                                                            */
/* CAUTION  : THIS PROGRAM IS ONLY FOR  " Q N X "                             */
/*                                                                            */
/* Auther   : Takahiro Baba                                                   */
/* Date     : 2002/10/28                                                      */
/* Version  : 0.5                                                             */
/* comment  : This server can receive, process and decode user commands       */
/*            sended by cpc(command processing client program).               */
/* See Also : Main Program (usually calling in main())                        */


#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>


#define		CPC_MSG_LEN		93

char		CPC_MY_IP[4] = {192,168,17,224} ; // vwpc10 192.168.17.212

int			cpflag = 1;

#include "function.h"

static resmgr_connect_funcs_t	connect_func;
static resmgr_io_funcs_t		io_func;
static iofunc_attr_t			attr;



void
end_process (void)
{
//    sleep(3);
    
}



void
process_data (int offset, char *buffer, int nbytes)
{

    int j;
    int ii,dd,cc;
    int debug = 1;
    unsigned char	*cpc_msg;
    char	cpc_target_ip[4];
    char	cpc_com[16];
    char	cpc_narg;
    long long	cpc_arg_i[8];
    double	cpc_arg_d[8];
    char	cpc_arg_c[8][8];
    unsigned char	cpc_arg_type[8];


//    if(debug) for(j=0;j<nbytes;j++) printf("cpc_msg[%2d] = %d\n",j,(unsigned char)buffer[j]);

        cpc_msg = (unsigned char *)malloc(CPC_MSG_LEN);
        memset (cpc_msg,0x00,sizeof(CPC_MSG_LEN));
        memcpy (cpc_msg, buffer, CPC_MSG_LEN);
        
        memcpy(cpc_target_ip, &cpc_msg[ 0],  4);
            if(strncmp(cpc_target_ip , CPC_MY_IP ,4) != 0) return;
        memcpy(cpc_com      , &cpc_msg[ 4], 16);if(debug) printf("CP > command : %s\n",cpc_com);
        memcpy(&cpc_narg    , &cpc_msg[20],  1);
        
        ii=0,dd=0,cc=0;
        
        for(j=0;j<cpc_narg;j++){
        
            memcpy(&cpc_arg_type[j+1], &cpc_msg[21+(9*j)],  1);

            if(cpc_arg_type[j+1] == 1){
                memcpy(&cpc_arg_i[ii], &cpc_msg[22+(9*j)],  8);
                ii++;
            }
            else if(cpc_arg_type[j+1] == 2){
                memcpy(&cpc_arg_d[dd], &cpc_msg[22+(9*j)],  8);
                dd++;
            }
            else if(cpc_arg_type[j+1] == 3){
                memcpy(cpc_arg_c[cc] , &cpc_msg[22+(9*j)],  8);
                cc++;
            }//else memset(&cpc_msg[22+(9*j)], 0x00, 8);
        
        }


    /*if(!(strncmp(cpc_com , "video1" ,6))){
                                          sleep(5);
                                          init();
                                          pthread_create (NULL, NULL, start, NULL);
                                          sleep(1);
                                          Dam();
                                          }
    if(!(strncmp(cpc_com , "video2" ,6))){
                                          sleep(5);
                                          init();
                                          pthread_create (NULL, NULL, start, NULL);
                                          sleep(1);
                                          Dam();
                                          CoR2();
                                          }
    */
    
    if(!(strncmp(cpc_com , "start"  ,5))) pthread_create (NULL, NULL, start, NULL);
    if(!(strncmp(cpc_com , "init"   ,4))) init();
    if(!(strncmp(cpc_com , "out"    ,3))) out(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "vel"    ,3))) vel(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "end"    ,3))) fin();
    if(!(strncmp(cpc_com , "fin"    ,3))) fin();
    if(!(strncmp(cpc_com , "exit"   ,4))) cpflag = 0;
    if(!(strncmp(cpc_com , "velome" ,6))) velome(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "tra"    ,3))) tra(cpc_arg_d[0]);
    if(!(strncmp(cpc_com , "tra2"   ,4))) tra2(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "tra3"   ,4))) tra3(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "tra4"   ,4))) tra4(cpc_arg_d[0]);
    if(!(strncmp(cpc_com , "tra5"   ,4))) tra5(cpc_arg_d[0]);
    if(!(strncmp(cpc_com , "tra6"   ,4))) tra6(cpc_arg_d[0]);
    if(!(strncmp(cpc_com , "mod5"   ,4))) mod5(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "mod4"   ,4))) mod4(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "modcas" , 6))) modcas(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "mod3"   ,4))) mod3(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "mod2"   ,4))) mod2(cpc_arg_d[0], cpc_arg_d[1], cpc_arg_d[2], cpc_arg_d[3], cpc_arg_d[4]);
    if(!(strncmp(cpc_com , "modonly",7))) modonly(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "modmod" ,6))) modmod(cpc_arg_d[0], cpc_arg_d[1]); 
	if(!(strncmp(cpc_com , "avo1"   ,4))) avo1(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "avo2"   ,4))) avo2(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "avo3"   ,4))) avo3(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "avo4"   ,4))) avo4(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "avo5"   ,4))) avo5(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "slope"  ,5))) slope(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "adtest" ,6))) adtest();
	if(!(strncmp(cpc_com , "lms"    ,3))) lms(cpc_arg_d[0], cpc_arg_d[1]);
    if(!(strncmp(cpc_com , "demo"    ,4))) demo(cpc_arg_d[0], cpc_arg_d[1]);
	if(!(strncmp(cpc_com , "obs"    ,3))) obs(cpc_arg_d[0], cpc_arg_d[1]);
}


int
io_write (resmgr_context_t *ctp, io_write_t *msg, iofunc_ocb_t *ocb)
{

    int		sts;
    int		nbytes;
    int		off;
    int		doffset;
    int		xtype;
    char	*buffer;
    struct _xtype_offset *xoffset;
    
    
    sts = iofunc_write_verify (ctp, msg, ocb, NULL);
    if ( sts != EOK ){
        return (sts);
    }
    
    
    xtype = msg->i.xtype & _IO_XTYPE_MASK;
    if ( xtype == _IO_XTYPE_MASK ){
        xoffset = (struct _xtype_offset *)(&msg->i+1);
        doffset = sizeof(msg->i) + sizeof(*xoffset);
        off = xoffset->offset;
    } else if ( xtype == _IO_XTYPE_NONE ){
        off = ocb->offset;
        doffset = sizeof(msg->i);
    } else return (ENOSYS);
    
    
    nbytes = msg->i.nbytes;
    if((buffer = malloc(nbytes)) == NULL) return (ENOSYS);
    
    
    if(resmgr_msgread(ctp, buffer, nbytes, doffset) == -1){
        free(buffer);
        return (errno);
    }
    
    process_data (off, buffer, nbytes);
    
    free (buffer);
    
    _IO_SET_WRITE_NBYTES (ctp, nbytes);
    
    if(nbytes){
        ocb->attr->flags |= IOFUNC_ATTR_MTIME
                         |  IOFUNC_ATTR_DIRTY_TIME;
        if(xtype == _IO_XTYPE_NONE){
            ocb->offset += nbytes;
        }
    }
    
    return (EOK);
}



int
io_open (resmgr_context_t *ctp, io_open_t *msg,
         RESMGR_HANDLE_T *handle, void *extra)
{
//    static int open_flag;
    
//    if (!open_flag){
//        open_flag = 1;
        printf ("CP > accessed.\n");
        return (iofunc_open_default (ctp, msg, handle, extra));
//    } else {
//        printf ("CP > accessed by other client\n");
//        printf ("CP > access denied\n");
//        return (-2);
//    }
    
}



int
io_close (resmgr_context_t *ctp, void *reserved,
         RESMGR_OCB_T *ocb)
{
    printf ("CP > closed.\n");
    return (iofunc_close_ocb_default (ctp, reserved, ocb));
}



int
io_read (resmgr_context_t *ctp, io_read_t *msg,
         iofunc_ocb_t *ocb)
{
    printf ("CP > ERROR: CP does not support \"read.\"\n");
    return (iofunc_read_default (ctp, msg, ocb));
}



//thread_pool_t *
int
cp (int argc, char **argv)
{

//    thread_pool_attr_t	pool_attr;
//    thread_pool_t			*tpp;
    dispatch_t				*dpp;
    resmgr_attr_t			resmgr_attr;
    resmgr_context_t		*ctp;
    int						id;
    int						cpc_id;
    
    if ((cpc_id = open("/dev/cpc", O_WRONLY)) != -1){
        printf ("/dev/cpc is already attached! \n");
        close(cpc_id);
        return (EXIT_FAILURE);
    }    
    
    if ((dpp = dispatch_create() ) == NULL){
        fprintf (stderr,"%s: Unable to dispatch_create.\n",argv[0]);
        return (EXIT_FAILURE);
    }
    
    
//    memset (&pool_attr, 0, sizeof (pool_attr));
//    
//    pool_attr.handle = dpp;
//    pool_attr.context_alloc = resmgr_context_alloc;
//    pool_attr.block_func = resmgr_block;
//    pool_attr.handler_func = resmgr_handler;
//    pool_attr.context_free = resmgr_context_free;
//    
    // set up the number of threads that you want
//    
//    pool_attr.lo_water = 0;
//    pool_attr.hi_water = 1;
//    pool_attr.increment = 1;
//    pool_attr.maximum = 10;
//
//    tpp = thread_pool_create (&pool_attr, POOL_FLAG_EXIT_SELF);
//
//    if (tpp == NULL){
//        fprintf (stderr,"%s: Unable to thread_pool_create.\n",argv[0]);
//        return (EXIT_FAILURE);
//    }
    
    iofunc_func_init (_RESMGR_CONNECT_NFUNCS, &connect_func,
                      _RESMGR_IO_NFUNCS, &io_func);
    
    iofunc_attr_init (&attr, S_IFNAM | 0777, 0, 0);
    
    // override functions in "connect_func" and "io_func" as required
    // (ex) connect_func.open = io_open;
    // (ex) io_func.io_read = io_read;
    
    connect_func.open = io_open;
    io_func.read = io_read;
    io_func.write = io_write;
    io_func.close_ocb = io_close;
    
    
    memset (&resmgr_attr, 0, sizeof(resmgr_attr));
    resmgr_attr.nparts_max = 1;
    resmgr_attr.msg_max_size = 2048;
    
    id = resmgr_attach (dpp, &resmgr_attr, "/dev/cpc", _FTYPE_ANY, _RESMGR_FLAG_BEFORE,
                        &connect_func, &io_func, &attr);
    
    if (id == -1){
        fprintf (stderr,"%s: Unable to resmgr_attach.\n",argv[0]);
        return (EXIT_FAILURE);
    }
    
    ctp = resmgr_context_alloc (dpp);
//    thread_pool_start (tpp);
    
    while(cpflag){
        
        if((ctp = resmgr_block (ctp)) == NULL){
            fprintf (stderr,"Unable to resmgr_block.\n");
            exit (EXIT_FAILURE);
        }
        
        resmgr_handler (ctp);
    }

    printf ("CP > cya!\n");
    return (EXIT_SUCCESS);
}



/ * / * To later generations. * /

/ * 1. About thread pool * /
/ * Currently, thread pool is not used. * /
/ * Server cannot be terminated using thread pool * /
/ * (The thread pool cannot be terminated). * /
/ * Two or more processes (command processes) at the same time in / dev / cpc * /
/ * Judgment that they will not overlap. * /
/ * Access from more than one client is not impossible. * /
/ * In fact, it can be accessed from more than one client. * /
/ * It's just a problem if you write at the same time (at atomic level). * /

/ * 2. About command processing * /
/ * As you can see, there is an embarrassing thing like strcmp in decoding the command. * /
/ * Beyond that it takes a lot of processing time and it is troublesome to write all commands * /
/ * Does not return a message that it is wrong for a wrong input command. * /
/ * The smartest way is for each command * /
/ * To execute pthread_create () using the received command character. * /
/ * However, using pthread_create () for the wrong command * /
/ * Don't know what will happen * /
/ * In the first place, I decided not to launch another thread. * /
/ * By switching the first character of the command * /
/ * The processing time may be improved as well. * /
/ * If you have come up with any good solution, please contact me. * /