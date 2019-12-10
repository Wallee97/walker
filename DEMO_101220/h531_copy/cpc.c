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

/* �X�V���������Ȃ�������B

0.5(02-10-28):cp�T�[�o�̑��d�N����h�~����@�\�����t�����B
�@�@�@�@�@�@�@���̂����ŁA���̒��ӎ����ɕύX����B
�@�@�@�@�@�@�@exit�R�}���h�ŁAexit�R�}���h�𑗐M����悤�ɕύX�B
�@�@�@�@�@�@�@fin�ł��A�����I��exit�R�}���h��ǉ��I�ɑ��M�B
�@�@�@�@�@�@�@�Ȃ�ł���Ȃ��Ƃ����Ȃ��Ⴂ���Ȃ����Ƃ����ƁA
�@�@�@�@�@�@�@cp.c�̕��ŁAio_close�ŏI���Ƃ������Z���g��Ȃ��������߁B
�@�@�@�@�@�@�@�܂��A���傤���Ȃ���ȁB
�@�@�@�@�@�@�@�����I���R�}���h��0.6�ōĒǉ�����錩�ʂ��B
�@�@�@�@�@�@�@�ʓ|�Ȃ̂ŁA�����A�ǉ�����Ȃ��B
�@�@�@�@�@�@�@�t���O�𗧂Ă�strcmp�P�ōς܂����Astrcmp���Q�����邩�B
*/

/* �R�}���h�Ɋւ��钍�ӎ���

�E64bit int�Ƃ���long long�^�Ȃ�Ă������̂��g�p���Ă��܂��B
�@�ilong long�^��C++�ł�64bit int�^�̈�ʓI�Ȍ^���A�炵���ł��j
�@qcc��64bit int��錾����X�}�[�g�Ȃ������킩��Ȃ���������ł��B
�@���ɂ��64bit int�̌^�͕ς��Ǝv���̂ŁA
�@�ł��邾�������A�X�}�[�g�ŃG���K���g�Ȃ�����T���Ă��������B
�@gcc���g���̂ł����Int64���g�����͂��ł��B
�Eexit�Ɠ��͂���ƃv���O�������I�����邱�ƂȂ�cpc�������I�����܂��B
�@�v���O�����{�̂Ƀf�[�^�͑��M����܂���B
�@cpc���I���������Ƃ��v���O�����{�̂��m�邽�߂ɂ́A
�@cp�f�o�C�X��o�^����Ƃ���io_close�֐������삵�Ă��������B
�Eexit�Ŏn�܂�R�}���h����͂����"exit"�ƔF������܂��B
�Eexit�ŏI�����Ă��A�Ă�cpc�𗧂��グ��Γ����͂��ł��B
�Efin�Ŏn�܂�R�}���h����͂����"fin"�Ƃ��ĔF������A
�@�v���O�����{�̂��I�����Acpc���I�����܂��B
�@�f�[�^�͑��M����܂��B
�E�ϐ��Ƃ��ĂW�����ȏ�̕��������͂����ꍇ�A
�@�ŏ��̂W���������͕ϐ��Ƃ��đ�����n�Y�ł��B
�E�T�P�P�����ȍ~�̓��̓f�[�^�͔j������܂��B
�@�o�b�t�@�I�[�o�[�t���[�΍�ł��B

*/

/* ���͂����f�[�^��ϐ��Ɏ�荞�ނƂ���Aunion���g���΁A8byte���ʂ̕ϐ���
memcpy����Ȃ�Ĕ������Ȃ����Ƃ����Ȃ��Ă��ςނ̂ɁA
��arcnet�̃h���C�o���������Ƃ��ɂ��v���܂������A
�ʓ|�Ȃ̂ŁA�㐶�̐l�����A�撣���Ă��������B
union�Ő錾����Ƃ���8byte�ϐ����g��Ȃ��ƁA���o�����ςɂȂ�܂��Bint�����ł����B
�����o���̎���8byte�ϐ���cast���Ă��ǂ������B�R���p�C���ɓ{����\����B
���ƁAunion�ȕϐ���錾������ɁA������malloc����memset��0�ɂ��Ȃ��ƃ��o�C�B
���ꂩ��o�b�t�@�I�[�o�[�t���[�Ƃ��ɂ��C��t���Ă��������B
�ꉞ�A����炵�����Ƃ͂��������ł��B*/
