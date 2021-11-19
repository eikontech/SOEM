/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

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

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

struct PositionOut
{
   int32 position;
   uint32 output;
   uint32 velocity_offset;
   uint16 control;
};
struct PositionIn
{
   int32 position;
   uint32 input;
   int32 velocity;
   uint16 status;
};

struct SpeedOut
{
   int32 speed;
   uint16 control;
};
struct SpeedIn
{
   int32 position;
   uint32 input;
   int32 speed;
   uint16 status;
};

/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)                                                                                                                      \
   {                                                                                                                                                               \
      buf = 0;                                                                                                                                                     \
      int __s = sizeof(buf);                                                                                                                                       \
      int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
      printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
   }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                       \
   {                                                                                                                                        \
      int __s = sizeof(buf);                                                                                                                \
      buf = value;                                                                                                                          \
      int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
      printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
   }

struct SpeedIn *val1 = NULL;
struct SpeedOut *target1 = NULL;

struct SpeedIn *val2 = NULL;
struct SpeedOut *target2 = NULL;

void redtest(char *ifname)
{
   int cnt, i, j, oloop, iloop;

   uint32 buf32;
   // uint16 buf16;
   uint8 buf8;

   printf("Starting Redundant test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */
      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);

         // for (int i = 1; i <= ec_slavecount; i++)
         // {
         //    printf("Has CA? %s\n", ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
         //    ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
         // }

         ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

         for (int i = 1; i <= ec_slavecount; i++)
         {
            WRITE(i, 0x6060, 0, buf8, 9, "OpMode"); // Operation mode velocity
            READ(i, 0x6061, 0, buf8, "OpMode display");
         }

         // int32 ob2;
         // int os;
         for (int i = 1; i <= ec_slavecount; i++)
         {
            WRITE(i, 0x1c12, 0, buf8, 0, "disable RX");
            WRITE(i, 0x1c12, 1, buf32, 0x1601, "assign 1601");
            WRITE(i, 0x1c12, 0, buf8, 1, "enable RX");

            WRITE(i, 0x1c13, 0, buf8, 0, "disable TX");
            WRITE(i, 0x1c13, 1, buf32, 0x1a03, "assign 1A03");
            WRITE(i, 0x1c13, 0, buf8, 1, "enable TX");
         }

         ec_config_map(&IOmap);

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

         /* configure DC options for every DC capable slave found in the list */
         ec_configdc();

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            ec_dcsync01(slave, TRUE, 125 * 1000, 0, 0);
         }

         /* read individual slave state and store in ec_slave[] */
         ec_readstate();
         for (cnt = 1; cnt <= ec_slavecount; cnt++)
         {
            printf("Slave:%d Name:\"%s\" Output size:%3dbits Input size:%3dbits State:%2d delay:%d. Has DC: %d\n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);

            // printf("\tOut:%.8x,%4d In:%.8x,%4d\n",
            //        (int)ec_slave[cnt].outputs[0], ec_slave[cnt].Obytes, (int)ec_slave[cnt].inputs[0], ec_slave[cnt].Ibytes);

            /* check for EL2004 or EL2008 */
            if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
            {
               digout = ec_slave[cnt].outputs;
            }
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);
         /* activate cyclic process data */
         dorun = 1;
         /* wait for all slaves to reach OP state */
         ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0))
            oloop = 1;
         if (oloop > 8)
            oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0))
            iloop = 1;
         if (iloop > 8)
            iloop = 8;

         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;

            for (int i = 1; i <= ec_slavecount; i++)
            {
               WRITE(i, 0x4602, 0, buf32, 1, "Release Brake");
               READ(i, 0x4602, 0, buf32, "Read Release Brake");
            }

            /* acyclic loop for slaves*/
            target1 = (struct SpeedOut *)(ec_slave[1].outputs);
            val1 = (struct SpeedIn *)(ec_slave[1].inputs);

            target2 = (struct SpeedOut *)(ec_slave[2].outputs);
            val2 = (struct SpeedIn *)(ec_slave[2].inputs);

            /* acyclic loop 5000 x 20ms = 10s */
            for (i = 1; i <= 5000; i++)
            {
               printf("Processdata cycle %5d , Wck %3d, DCtime %12ld, dt %12ld, O:",
                      dorun, wkc, ec_DCtime, gl_delta);
               for (j = 0; j < oloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].outputs + j));
               }
               printf(" I:");
               for (j = 0; j < iloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].inputs + j));
               }
               printf("\r");
               fflush(stdout);

               osal_usleep(20000);
            }
            dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End redundant test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
   }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if (ts->tv_nsec > NSEC_PER_SEC)
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime;
   }
   if (delta > 0)
   {
      integral++;
   }
   if (delta < 0)
   {
      integral--;
   }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec ts, tleft;
   int ht;
   int64 cycletime;

   // uint32 buf32;
   // // uint16 buf16;
   // uint8 buf8;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = *(int *)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   uint counter = 0;
   int blink = 0;

   ec_send_processdata();
   while (1)
   {
      counter++;
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun > 0)
      {
         wkc = ec_receive_processdata(EC_TIMEOUTRET);

         if (wkc >= expectedWKC && val1 && target1)
         {
            if ((val1->status & 0b0000000001001111) == 0b0000000000000000)
            { // Not ready to switch on
               printf("Error: transition Not Ready to Switch On => Switch On Disabled should be automatic");
            }
            else if ((val1->status & 0b0000000001001111) == 0b0000000001000000)
            {                         // Switch on disabled
               target1->control = 6U; // transition 2
            }
            else if ((val1->status & 0b0000000001101111) == 0b0000000000100001)
            {                         // Ready to switch on
               target1->control = 7U; // transition 3
            }
            else if ((val1->status & 0b0000000001101111) == 0b0000000000100011)
            {                          // Switched on
               target1->control = 15U; // transition 4
            }
            else if ((val1->status & 0b0000000001001000) == 0b0000000000001000)
            { // Fault + Fault reaction active
               // READ(1, 0x1001, 0, buf8, "Error");
               // READ(1, 0x1003, 0, buf32, "# of Error");
               // int max_err = buf32;
               // for (int j = 1; j <= max_err; ++j)
               // {
               //    char label[8];
               //    sprintf(label, "Error %d", j);
               //    READ(1, 0x1003, j, buf32, label);
               // }

               target1->control = 128U; // transition 15
            }

            if ((val1->status & 0b0000000001101111) == 0b0000000000100111) // Operation enabled
            {
               target1->speed = 50000;
               target1->speed *= blink? -1:1;
               if (counter % 10000 == 0)
               {
                  blink = !blink;
               }
               printf("slave 1 --> counter %d, speed: %d\n", counter, target1->speed);

            }

            if ((val2->status & 0b0000000001001111) == 0b0000000000000000)
            { // Not ready to switch on
               printf("Error: transition Not Ready to Switch On => Switch On Disabled should be automatic");
            }
            else if ((val2->status & 0b0000000001001111) == 0b0000000001000000)
            {                         // Switch on disabled
               target2->control = 6U; // transition 2
            }
            else if ((val2->status & 0b0000000001101111) == 0b0000000000100001)
            {                         // Ready to switch on
               target2->control = 7U; // transition 3
            }
            else if ((val2->status & 0b0000000001101111) == 0b0000000000100011)
            {                          // Switched on
               target2->control = 15U; // transition 4
            }
            else if ((val2->status & 0b0000000001001000) == 0b0000000000001000)
            { // Fault + Fault reaction active
               // READ(1, 0x1001, 0, buf8, "Error");
               // READ(1, 0x1003, 0, buf32, "# of Error");
               // int max_err = buf32;
               // for (int j = 1; j <= max_err; ++j)
               // {
               //    char label[8];
               //    sprintf(label, "Error %d", j);
               //    READ(1, 0x1003, j, buf32, label);
               // }

               target2->control = 128U; // transition 15
            }

            if ((val2->status & 0b0000000001101111) == 0b0000000000100111) // Operation enabled
            {
               target2->speed = 50000;
               target2->speed *= blink? 1:-1;
               printf("slave 2 --> counter %d, speed: %d\n", counter, target2->speed);

            }

            // if ((val1->status & 0b0000000001101111) == 0b0000000000100111 /*&& reachedInitial*/) // Operation enabled
            // {
            //    // printf("set position.\n");
            //    target1->position = 2000000;
            //    target1->velocity_offset = 500;
            // }
            needlf = TRUE;
         }

         dorun++;
         /* if we have some digital output, cycle */
         if (digout)
            *digout = (uint8)((dorun / 16) & 0xff);

         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         ec_send_processdata();
      }
   }
}

OSAL_THREAD_FUNC ecatcheck()
{
   int slave;

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
   int ctime;

   printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

   if (argc > 2)
   {
      dorun = 0;
      ctime = atoi(argv[2]);

      /* create RT thread */
      osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime);

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

      /* start acyclic part */
      redtest(argv[1]);
   }
   else
   {
      printf("Usage: red_test ifname1 cycletime\nifname = eth0 for example\ncycletime in us\n");
   }

   printf("End program\n");

   return (0);
}
