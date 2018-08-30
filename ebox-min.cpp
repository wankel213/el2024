#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "ethercat.h"

#define NameState(X) (X == EC_STATE_INIT)? "EC_STATE_INIT" : \
                    (X == EC_STATE_PRE_OP)? "EC_STATE_PRE_OP" : \
                    (X == EC_STATE_BOOT)? "EC_STATE_BOOT" : \
                    (X == EC_STATE_SAFE_OP)? "EC_STATE_SAFE_OP" : \
                    (X == EC_STATE_OPERATIONAL)? "EC_STATE_OPERATIONAL" : \
                    (X == EC_STATE_ERROR)? "EC_STATE_ERROR | ACK" : "EC_STATE_NONE" \



// total samples to capture
#define MAXSTREAM 200000
// sample interval in ns, here 8us -> 125kHz
// maximum data rate for E/BOX v1.0.1 is around 150kHz
#define SYNC0TIME 8000

#define NSEC_PER_SEC 1000000000


// -------------- structures ---------------------

namespace bag {
    typedef struct PACKED {
        uint8         status;
        uint8         counter;
        uint8         din;
        int32         ain[2];
        uint32        tsain;
        int32         enc[2];
    } in_EBOXt;

    typedef struct PACKED {
        uint8         control;
        uint8         dout;
        int16         aout[2];
        uint16        pwmout[2];
    } out_EBOXt;

}


// ---------- globals -------------------

char IOmap[4096];
int dorun = 0;
int64 integral=0;
uint32 cyclecount=0;

bag::in_EBOXt  *in_EBOX;
bag::out_EBOXt *out_EBOX;

double     ain[2];
int        ainc;

pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// --------------------------------------


void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime /2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral /20);
}


void* ecatthread( void *ptr ) {

    printf("ethercat thread\n");

//    struct timespec   ts;
//    struct timeval    tp;
//    int ht;
//    int i;
//    int pcounter = 0;
   int64 cycletime;

//    int64 toff;
//     int DCdiff;

   pthread_mutex_lock(&mutex);
//    gettimeofday(&tp, NULL);

    /* Convert from timeval to timespec */
//    ts.tv_sec  = tp.tv_sec;
//    ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
//    ts.tv_nsec = ht * 1000000;
//    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
//    toff = 0;
//    dorun = 0;
   while(1)
   {
      /* calculate next cycle start */
    //   add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
    //   pthread_cond_timedwait(&cond, &mutex, &ts);
      if (dorun>0)
      {
        //  gettimeofday(&tp, NULL);

         // TODO
         out_EBOX->aout[0] = 0x3FFF;
         out_EBOX->aout[1] = 0x3FFF;
         out_EBOX->dout = 0b01010101;

         ec_send_processdata();

         ec_receive_processdata(EC_TIMEOUTRET);

         cyclecount++;

        //  pcounter = in_EBOX->counter;
         /* calulate toff to get linux time and DC synced */
        //  ec_sync(ec_DCtime, cycletime, &toff);
      }
   }
}

// specific to ebox
void eboxStart (){

    printf("ebox start\n");

    int ctime = 1000;

    pthread_t thread1;

    struct sched_param schedp;
    struct sched_param param;

    memset(&schedp, 0, sizeof(schedp));
    memset(&param, 0, sizeof(param));

    /* do not set priority above 49, otherwise sockets are starved */
    schedp.sched_priority = 30;
    param.sched_priority = 40;

    // set current thread priority
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // Create real-time thread with higher  priority
    pthread_create( &thread1, NULL, &ecatthread, (void *) &ctime);
    pthread_setschedparam(thread1, SCHED_OTHER, &param);


    // ----------------------------------------------

    printf("Number of slaves: %d\n", ec_slavecount);
    printf("Ebox state: %s\n", NameState(ec_slave[1].state));

    int os;
    uint32 ob;
    int16 ob2;
    uint8 ob3;

    /* send one processdata cycle to init SM in slaves */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    printf("Ebox state: %s\n", NameState(ec_slave[1].state));

    // Change state of slaves
    ec_slave[0].state = EC_STATE_PRE_OP; // set global state for all slaves
    ec_writestate(0); // write state for all slaves
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE); // wait for all slaves to change state

    // TODO: set configurations for the slaves
    printf("Ebox state: %s\n", NameState(ec_slave[1].state));

    ec_config_map(&IOmap);

    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

    ec_configdc();

    if (( ec_slavecount >= 1 )) {
        // reprogram PDO mapping to set slave in stream mode
        // this can only be done in pre-OP state
        for(int i=1; i <= ec_slavecount; i++) {
            if(strcmp(ec_slave[i].name,"E/BOX") == 0){

                printf("Ebox state: %s\n", NameState(ec_slave[i].state));

                 /* connect struct pointers to slave I/O pointers */
                in_EBOX = (bag::in_EBOXt*) ec_slave[i].inputs;
                out_EBOX = (bag::out_EBOXt*) ec_slave[i].outputs;

                /* read indevidual slave state and store in ec_slave[] */
                ec_readstate();

                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%s delay:%d.%d\n",
                     i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                     NameState(ec_slave[i].state), (int)ec_slave[i].pdelay, ec_slave[i].hasdc);

                /* send one processdata cycle to init SM in slaves */
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                // Change state of slaves
                ec_slave[0].state = EC_STATE_OPERATIONAL; // set global state for all slaves
                ec_writestate(0); // write state for all slaves
                ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE); // wait for all slaves to change state

                printf("Ebox state: %s\n", NameState(ec_slave[i].state));

                if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
                //     printf("Operational state reached for all slaves.\n");
                    // ain[0] = 0;
                    // ain[1] = 0;
                    // ainc = 0;
                    dorun = 1;
                    // usleep(100000); // wait for linux to sync on DC
                    // ec_dcsync0(1, TRUE, SYNC0TIME, 0); // SYNC0 on slave 1
                }

                while(1) { 
                    i++;
                     printf("PD cycle %5d DCtime %12ld Cnt:%3d Status:%2u Digital In: %#4x Analog in: %6d %6d Digital out: %#4x Analog out: %6d %6d\n",
                            cyclecount, ec_DCtime, in_EBOX->counter, in_EBOX->status,
                            in_EBOX->din,
                            in_EBOX->ain[0], in_EBOX->ain[1],
                            out_EBOX->dout,
                            out_EBOX->aout[0], out_EBOX->aout[1]);
                    usleep(20000);
                }
                dorun = 0;
            }
        }
    }


    ec_close();
    
}


int main(){
    
    if (ec_init("enp6s0")) {
        printf("ec_init succeeded.\n");

        /* find and auto-config slaves */
        ec_config_init(FALSE);
        // if ( ec_config_init(FALSE) > 0 ){ // does not populate IO bit size
        if ( ec_config(FALSE, &IOmap) > 0 ) { // populates IO bit size
            ec_configdc();
            printf("%d slaves found and configured.\n",ec_slavecount);

            ec_readstate();

            for(int i=1; i<=ec_slavecount; i++){
                printf("Slave: %d\n\t Name: %s\n",i, ec_slave[i].name);
                printf("\t Configured address: %4.4x\n", ec_slave[i].configadr);
                printf("\t Output size: %dbits\n\t Input size: %dbits\n\t State: %s\n\t Delay: %d[ns]\n\t Has DC: %d\n",
                  ec_slave[i].Obits, ec_slave[i].Ibits,
                  NameState(ec_slave[i].state), ec_slave[i].pdelay, ec_slave[i].hasdc);
            }
        }
        eboxStart();
        ec_close();
    }

    return 0;
}