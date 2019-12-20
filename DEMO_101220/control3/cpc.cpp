/* for wh4 cpc.c    : Command Processing Client                                       */
/*                                                                            */
/* CAUTION  : THIS PROGRAM IS ONLY FOR  " Q N X "                             */
/*                                                                            */
/* Auther   : Takahiro Baba                                                   */
/* Date     : 2002/10/28                                                      */
/* Version  : 0.5                                                             */
/* comment  : This client can get input command,                              */
/*            process and decode user commands,                               */
/*            and send to CP server(command processing server).               */
/* CAUTION  : This code should compile independently.                         */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/uio.h>

#define		CPC_MSG_LEN		93

char	CPC_TARGET_IP[4] = {192,168,17,224} ; // vwpc10 192.168.17.212


int debug = 0;



int
main()
{

    int		cpc;
    int		j,len[8];
    int		i,d,c;
    int		ii,dd,cc;
    int		flag = 0;
    int		point;
    int		sp;
    int		str_len;
    char	input[512];
    char	buff[127];
    int		cpc_flag[10];
    char	cpc_com[16];
    char	cpc_narg;
    long long	cpc_arg_i[8];// 64bit integer
    double	cpc_arg_d[8];
    char	cpc_arg_c[8][8];
    unsigned char	cpc_arg_type[8];
    unsigned char	*cpc_msg;


//    if(!debug) cpc = open("/dev/cpc", O_WRONLY);

    if ((cpc = open("/dev/cpc", O_WRONLY)) == -1){
        printf ("CPC > cannot access to CP server!\n");
        return (EXIT_FAILURE);
    }    

    printf("CPC > access ok\n");

    cpc_msg = (unsigned char *)malloc(CPC_MSG_LEN);

    while(1){
        
        memset(cpc_com,0x00,sizeof(cpc_com));
        memset(cpc_arg_c,0x00,sizeof(cpc_arg_c));
        memset(input,0x00,sizeof(input));
        
        for(j=0;j<10;j++){
            cpc_flag[j] = 0;
        }
        cpc_flag[0]=1;
        
        for(j=0;j<8;j++){
            cpc_arg_i[j] = 0;
            cpc_arg_d[j] = 0.0;
            cpc_arg_type[j] = 0;
        }
        
        flag = 0;
        cpc_narg = 0;
        i=0;ii=0;d=0;dd=0;c=0;cc=0;
        
        
        printf("\nCPC > ");
        fgets(input,511,stdin);
        
        str_len = strlen(input);
        
        for(j=1;j<str_len+1;j++){
            if(cpc_flag[flag]){
                if(input[j] == 32 || input[j] == 40 || input[j] == 0){
                    len[0] = j;
                    point = j + 1;
                    memcpy(cpc_com,input,len[0]);
                    cpc_flag[flag] = 0;
                    if(input[j] != 0) cpc_flag[1] = 1;
                }
            }
        }
        
        for(flag=1;flag<9;flag++){
            sp=1;
            if(cpc_flag[flag]){
                for(j=point;j<str_len+1;j++){
                    if(cpc_arg_type[flag] == 0){
                        if(input[j] == 34) cpc_arg_type[flag] = 3;
                        if(input[j] == 46) cpc_arg_type[flag] = 2;
                    }
                    if(input[j] == 40) point++;
                    if((sp == 1) && (input[j] == 32) || (sp == 1) && (input[j] == 44) ){
                        point++;
                    } else sp = 0;
                    if(((sp == 0) && (input[j] == 32)) || ((sp == 0) && (input[j] == 44)) || input[j] == 0 || input[j] == 41){

                        len[flag] = j - point;
                        
                        if(cpc_arg_type[flag] == 3){
                            memcpy(cpc_arg_c[c],&input[point+1],len[flag]-2);
                            c++;
                        } else if(cpc_arg_type[flag] == 2){
                            memset(buff,0x00,sizeof(buff));
                            memcpy(buff,&input[point],len[flag]);
                            cpc_arg_d[d] = atof(buff);
                            d++;
                        } else if(len[flag] != 0){
                            cpc_arg_type[flag] = 1;
                            memset(buff,0x00,sizeof(buff));
                            memcpy(buff,&input[point],len[flag]);
                            cpc_arg_i[i] = atoi(buff);
                            i++;
                        } else{
                            cpc_flag[flag] = 0;
                            break;
                        }

                    cpc_narg++;
                    point += len[flag];
                    point++;
                    
                    cpc_flag[flag] = 0;
                    if(input[j] == 32 || input[j]  == 44) cpc_flag[flag+1] = 1;
                    
                    break;

                    }
                }
            }
        }
        
        if(cpc_narg > 8 || cpc_narg < 0) break;
        
        if(debug){
            printf("\n command = %s
                    \n cpc_narg = %d\n",cpc_com,cpc_narg);
            
            for(j=1;j<9;j++){
                if(cpc_arg_type[j] == 1){
                    printf("\n cpc_arg_type[%d] = int
                            \n cpc_arg_i[%d] = %d\n",j,j,cpc_arg_i[ii]);
                    ii++;
                } else if(cpc_arg_type[j] == 2){
                    printf("\n cpc_arg_type[%d] = double
                            \n cpc_arg_d[%d] = %f\n",j,j,cpc_arg_d[dd]);
                    dd++;
                } else if(cpc_arg_type[j] == 3){
                    printf("\n cpc_arg_type[%d] = char *
                            \n cpc_arg_c[%d] = %s\n",j,j,cpc_arg_c[cc]);
                    cc++;
                } else break;
            }
        
        }// end debug
        
        if(!(strncmp(cpc_com , "exit" ,4))) break;
        
        // cpc message setting
        
        memset(cpc_msg,0x00,sizeof(CPC_MSG_LEN));
        
        memcpy(&cpc_msg[ 0], CPC_TARGET_IP,  4);
        memcpy(&cpc_msg[ 4], cpc_com      , 16);
        memcpy(&cpc_msg[20], &cpc_narg    ,  1);
        
        ii=0,dd=0,cc=0;
        
        for(j=0;j<cpc_narg;j++){
        
            memcpy(&cpc_msg[21+(9*j)], &cpc_arg_type[j+1] ,  1);

            if(cpc_arg_type[j+1] == 1){
                memcpy(&cpc_msg[22+(9*j)], &cpc_arg_i[ii],  8);
                ii++;
            }
            else if(cpc_arg_type[j+1] == 2){
                memcpy(&cpc_msg[22+(9*j)], &cpc_arg_d[dd],  8);
                dd++;
            }
            else if(cpc_arg_type[j+1] == 3){
                memcpy(&cpc_msg[22+(9*j)], cpc_arg_c[cc] ,  8);
                cc++;
            }//else memset(&cpc_msg[22+(9*j)], 0x00, 8);
        
        }
        
        if(debug) for(j=0;j<CPC_MSG_LEN;j++) printf("cpc_msg[%2d] = %d\n",j,cpc_msg[j]);
        
        // end setting
        
        if(!debug){
            if(write(cpc,cpc_msg,CPC_MSG_LEN) != CPC_MSG_LEN) 
                                  printf("CPC > ... ERROR!\n");
            else printf("CPC > ... OK\n");
        }

        if(!(strncmp(cpc_com , "fin" ,3))) break;

    }// end while(1)

    memset(cpc_msg,0x00,sizeof(CPC_MSG_LEN));
    memcpy(&cpc_msg[ 0], CPC_TARGET_IP,  4);
    memcpy(&cpc_msg[ 4], "exit"       ,  4);

    write(cpc,cpc_msg,CPC_MSG_LEN);

    close(cpc);

    free(cpc_msg);
    
    printf("CPC > Bye!\n");
}

/* What is the update history?

0.5 (02-10-28): A mechanism to prevent multiple activation of cp server was added.
Because of that, the notes below have been changed.
Changed to send the exit command with the exit command.
Even in fin, the exit command is automatically sent additionally.
The reason why we have to do this is:
ÇΩ Çﬂ Because cp.c no longer uses the trick of ending with io_close.
Well, I ca nÅft help it.
The forced termination command is expected to be re-added with 0.6.
Since it is cumbersome, maybe not added.
Whether to set a flag and use one strcmp or add two strcmp.
*/

/* Notes on commands

ÅE Long long type is used as 64bit int.
(Long long type seems to be a common type name of 64bit int type in C ++)
Ç© ÇÁ I didn't know the smart way to declare 64bit int in qcc.
Since the type of 64-bit int will change depending on the environment,
Please look for a smart and elegant way as soon as possible.
If you use gcc, you should have been able to use Int64.
ÅE Enter exit to forcibly terminate cpc without exiting the program.
* Data is not sent to the program.
In order for the program body to know that cpc has ended,
* Create your own io_close function for registering cp devices.
-If you enter a command starting with exit, it will be recognized as "exit".
-Even if you exit with exit, it should work if you start cpc again.
-If you enter a command starting with fin, it will be recognized as "fin"
Quit the program body and ccp.
Data will be sent.
ÅE If you enter a string of 8 or more characters as a variable,
The first 8 characters are sent as input variables.
ÅE Input data after 511 characters will be discarded.
Measures against buffer overflow.

*/

/* When input data is stored in a variable, if union is used, 8 bytes are stored in another variable
You do nÅft have to do anything beautiful to memcpy,
I thought when I wrote the arcnet driver,
Since it is troublesome, please do your best for the later generations.
If you do not use 8byte variables when declaring in union, the retrieval will be strange. Only int.
May be cast to 8byte variable when exporting. The possibility of getting angry with the compiler.
Also, after declaring a union variable, you must malloc it properly and set it to 0 with memset.
Also watch out for buffer overflows.
I intend to do something like that. */
