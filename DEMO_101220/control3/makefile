##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w9
OBJ=main.o ../driver_qnx/aclad/AD_driver.o ../driver_qnx/aclda/DA_driver.o ../driver_qnx/hpc/Cnt_driver.o cp.o timer.o 
LIBS= -lm -lsocket
#LIBS = -lm
all: main

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

../driver_qnx/hpc/Cnt_driver.o: ../driver_qnx/hpc/Cnt_driver.c ../driver_qnx/hpc/hpc-ctrdriver.h
../driver_qnx/aclda/DA_driver.o: ../driver_qnx/aclda/DA_driver.c ../driver_qnx/aclda/acl8112dadriver.h
../driver_qnx/aclad/AD_driver.o: ../driver_qnx/aclad/AD_driver.c ../driver_qnx/aclad/acl8112addriver.h

main.o: main.c ../driver_qnx/aclda/acl8112dadriver.h ../driver_qnx/hpc/hpc-ctrdriver.h ../driver_qnx/aclad/acl8112addriver.h

cp.o: cp.c
timer.o: timer.c timer.h

main: $(OBJ) 
	$(CC) -o main $(OBJ) $(LIBS)






