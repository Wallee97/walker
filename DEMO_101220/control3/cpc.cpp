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

/* 更新履歴つうかなんっつうか。

0.5(02-10-28):cpサーバの多重起動を防止する機構を取り付けた。
　　　　　　　そのせいで、下の注意事項に変更あり。
　　　　　　　exitコマンドで、exitコマンドを送信するように変更。
　　　　　　　finでも、自動的にexitコマンドを追加的に送信。
　　　　　　　なんでこんなことをしなきゃいけないかというと、
　　　　　　　cp.cの方で、io_closeで終了という裏技を使わなくしたため。
　　　　　　　まあ、しょうがないわな。
　　　　　　　強制終了コマンドは0.6で再追加される見通し。
　　　　　　　面倒なので、多分、追加されない。
　　　　　　　フラグを立ててstrcmp１個で済ますか、strcmpを２個加えるか。
*/

/* コマンドに関する注意事項

・64bit intとしてlong long型なんていうものを使用しています。
　（long long型はC++での64bit int型の一般的な型名、らしいです）
　qccで64bit intを宣言するスマートなやり方がわからなかったからです。
　環境により64bit intの型は変わると思うので、
　できるだけ早く、スマートでエレガントなやり方を探してください。
　gccを使うのであればInt64が使えたはずです。
・exitと入力するとプログラムを終了することなくcpcを強制終了します。
　プログラム本体にデータは送信されません。
　cpcが終了したことをプログラム本体が知るためには、
　cpデバイスを登録するときのio_close関数を自作してください。
・exitで始まるコマンドを入力すると"exit"と認識されます。
・exitで終了しても、再びcpcを立ち上げれば動くはずです。
・finで始まるコマンドを入力すると"fin"として認識され、
　プログラム本体を終了し、cpcを終了します。
　データは送信されます。
・変数として８文字以上の文字列を入力した場合、
　最初の８文字が入力変数として送られるハズです。
・５１１文字以降の入力データは破棄されます。
　バッファオーバーフロー対策です。

*/

/* 入力したデータを変数に取り込むところ、unionを使えば、8byteずつ別の変数に
memcpyするなんて美しくないことをしなくても済むのに、
とarcnetのドライバを書いたときにも思いましたが、
面倒なので、後生の人たち、頑張ってください。
unionで宣言するときに8byte変数を使わないと、取り出しが変になります。intだけですが。
書き出しの時に8byte変数にcastしても良いかも。コンパイラに怒られる可能性大。
あと、unionな変数を宣言した後に、ちゃんとmallocしてmemsetで0にしないとヤバイ。
それからバッファオーバーフローとかにも気を付けてください。
一応、それらしいことはやったつもりです。*/
