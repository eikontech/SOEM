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
pthread_t thread1, thread2, thread3;
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

int operation_mode = 9;

typedef struct
{
   int32 position;
   int32 velocity;
   int16 torque;
   uint16 maxTorque;
   uint16 control;
   uint8 opMode;
} ControlOut;

typedef struct
{
   uint16 errorCode;
   uint16 status;
   int32 position;
   int32 velocity;
   int16 torque;
   uint8 modeDisplay;
} ControlIn;

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

ControlIn *val[EC_MAXSLAVE];
ControlOut *target[EC_MAXSLAVE];

void redtest(char *ifname)
{
   int oloop, iloop;

   uint32 buf32;
   uint16 buf16;
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

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            WRITE(slave, 0x6060, 0, buf8, operation_mode, "OpMode"); // Operation mode position
            READ(slave, 0x6061, 0, buf8, "OpMode display");

            // WRITE(slave, 0x6081, 0, buf32, 100000, "Set Contour Speed");
            // WRITE(slave, 0x6083, 0, buf32, 5566, "Set Profile Acceleration");
            // WRITE(slave, 0x6084, 0, buf32, 5566, "Set profile Deceleration");

            // READ(slave, 0x6081, 0, buf32, "Read Contour Speed");
            // READ(slave, 0x6083, 0, buf32, "Read Profile Acceleration");
            // READ(slave, 0x6084, 0, buf32, "Read profile Deceleration");

            // READ(slave, 0x6064, 0, buf32, "Read Actual Position");
            // osal_usleep(EC_TIMEOUTTXM);
            // int32_t actual_location = buf32;
            // char label[60];
            // sprintf(label, "Set Actual Position %d", actual_location);
            // WRITE(slave, 0x607A, 0, buf32, actual_location, label);
            // osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6081, 0, buf32, 100000, "Set Profile Velocity");
            osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6083, 0, buf32, 5566, "Set Profile Aceleration");
            osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6084, 0, buf32, 5566, "Set Profile Deceleration");
            osal_usleep(EC_TIMEOUTTXM);

            READ(slave, 0x607A, 0, buf32, "Read Target Location");
            READ(slave, 0x6064, 0, buf32, "Read Actual Location");
            READ(slave, 0x6041, 0, buf8, "Read Status Word");
         }

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            WRITE(slave, 0x1c12, 0, buf8, 0, "disable RX");
            WRITE(slave, 0x1c12, 1, buf16, 0x1605, "assign 1605");
            WRITE(slave, 0x1c12, 0, buf8, 1, "enable RX");
            osal_usleep(EC_TIMEOUTTXM);
            READ(slave, 0x1c12, 1, buf16, "read 1c12/1");

            READ(slave, 0x1605, 0, buf8, "read 1605")
            int elems = buf8;
            for (int j = 1; j <= elems; ++j)
            {
               char label[8];
               sprintf(label, "field %d", j);
               READ(slave, 0x1605, j, buf32, label);
            }
            READ(slave, 0x1c12, 1, buf16, "read 1c12/1");

            WRITE(slave, 0x1c13, 0, buf8, 0, "disable TX");
            WRITE(slave, 0x1c13, 1, buf32, 0x1a06, "assign 1A06");
            WRITE(slave, 0x1c13, 0, buf8, 1, "enable TX");
         }

         ec_config_map(&IOmap);

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

         /* configure DC options for every DC capable slave found in the list */
         ec_configdc();

         /* activate cyclic process data */
         dorun = 1;

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            ec_dcsync0(slave, TRUE, 125 * 1000, 0);
         }

         /* read individual slave state and store in ec_slave[] */
         ec_readstate();
         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            printf("Slave:%d Name:\"%s\" Output size:%3dbits Input size:%3dbits State:%2d delay:%d. Has DC: %d\n",
                   slave, ec_slave[slave].name, ec_slave[slave].Obits, ec_slave[slave].Ibits,
                   ec_slave[slave].state, (int)ec_slave[slave].pdelay, ec_slave[slave].hasdc);

            // printf("\tOut:%.8x,%4d In:%.8x,%4d\n",
            //        (int)ec_slave[slave].outputs[0], ec_slave[slave].Obytes, (int)ec_slave[slave].inputs[0], ec_slave[slave].Ibytes);

            /* check for EL2004 or EL2008 */
            if (!digout && ((ec_slave[slave].eep_id == 0x0af83052) || (ec_slave[slave].eep_id == 0x07d83052)))
            {
               digout = ec_slave[slave].outputs;
            }
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         /* I would then wait at least 1 second before going to OP */
         osal_usleep(1000000);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);

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

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
               WRITE(slave, 0x4602, 0, buf32, 1, "Release Brake");
               READ(slave, 0x4602, 0, buf32, "Read Release Brake");

               /* acyclic loop for slaves*/
               target[slave] = (ControlOut *)(ec_slave[slave].outputs);
               val[slave] = (ControlIn *)(ec_slave[slave].inputs);
            }

            /* acyclic loop 5000 x 20ms = 10s */
            for (int i = 1; i <= 5000; i++)
            {
               // printf("Processdata cycle %5d , Wck %3d, DCtime %12ld, dt %12ld, O:",
               //        dorun, wkc, ec_DCtime, gl_delta);
               // for (int j = 0; j < oloop; j++)
               // {
               //    printf(" %2.2x", *(ec_slave[0].outputs + j));
               // }
               // printf(" I:");
               // for (int j = 0; j < iloop; j++)
               // {
               //    printf(" %2.2x", *(ec_slave[0].inputs + j));
               // }
               // printf("\r");
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
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
               if (ec_slave[slave].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
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

int error_cumulative = 0;
int kp = 500;
int ki = 70;

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec ts, tleft;
   int ht;
   int64 cycletime;

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
         ec_send_processdata();
         wkc = ec_receive_processdata(EC_TIMEOUTRET);

         if (wkc >= expectedWKC)
         {
            //  && val && target)
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
               if (val[slave] && target[slave])
               {
                  if ((val[slave]->status & 0b0000000001001111) == 0b0000000000000000)
                  { // Not ready to switch on
                     printf("Error: transition Not Ready to Switch On => Switch On Disabled should be automatic, %d.\n", val[slave]->status);
                  }
                  else if ((val[slave]->status & 0b0000000001001111) == 0b0000000001000000)
                  { // Switch on disabled
                     // printf("Switch on disabled, %d.\n", val[slave]->status);
                     target[slave]->control = 6U; // transition 2
                  }
                  else if ((val[slave]->status & 0b0000000001101111) == 0b0000000000100001)
                  { // Ready to switch on
                     // printf("Ready to switch on, %d.\n", val[slave]->status);
                     target[slave]->control = 7U; // transition 3
                  }
                  else if ((val[slave]->status & 0b0000000001101111) == 0b0000000000100011)
                  { // Switched on
                     // printf("Switched on, %d.\n", val[slave]->status);
                     target[slave]->control = 15U; // transition 4
                  }
                  else if ((val[slave]->status & 0b0000000001001000) == 0b0000000000001000)
                  { // Fault + Fault reaction active
                     // READ(1, 0x1001, 0, buf8, "Error");
                     int buf32;
                     READ(1, 0x1003, 0, buf32, "# of Error");
                     int max_err = buf32;
                     for (int j = 1; j <= max_err; ++j)
                     {
                        char label[8];
                        sprintf(label, "Error %d", j);
                        READ(1, 0x1003, j, buf32, label);
                     }

                     target[slave]->control = 128U; // transition 15
                  }

                  if (operation_mode == 8)
                     target[slave]->opMode = 9;
                  else
                     target[slave]->opMode = operation_mode;

                  if ((val[slave]->status & 0b0000000001101111) == 0b0000000000100111) // Operation enabled
                  {
                     target[slave]->position = 50000;
                     target[slave]->position *= blink ? -1 : 1;

                     if (operation_mode == 8)
                     {
                        int error = target[slave]->position - val[slave]->position;
                        if (abs(error) < 50)
                        {
                           error_cumulative = 0;
                           target[slave]->velocity = 0;
                        }
                        else
                        {
                           target[slave]->velocity = kp * error + ki * error_cumulative;
                           error_cumulative += error;
                           if (abs(target[slave]->velocity) > 100000)
                              target[slave]->velocity = target[slave]->velocity > 0 ? 100000 : -100000;
                        }
                     }
                     else
                     {
                        target[slave]->velocity = 50000;
                        target[slave]->velocity *= blink ? -1 : 1;
                     }

                     target[slave]->torque = 250;
                     // target[slave]->torque *= blink ? -1 : 1;

                     if (counter % 10000 == 0)
                     {
                        blink = !blink;
                        printf("counter %d\n", counter);
                        printf("target:\tposition:\t%d,\t\tspeed:\t%d,\ttorque:\t%d,\topMode:\t%d\n",
                               target[slave]->position, target[slave]->velocity, target[slave]->torque, target[slave]->opMode);
                        printf("actual:\tposition:\t%d,\tspeed:\t%d,\ttorque:\t%d,\topMode:\t%d\n",
                               val[slave]->position, val[slave]->velocity, val[slave]->torque, val[slave]->modeDisplay);
                        printf("Error code %d\n", val[slave]->errorCode);
                     }
                     // printf("actual:\tposition:\t%d,\tspeed:\t%d,\ttorque:\t%d,\topMode:\t%d\n",
                     //        val[slave]->position, val[slave]->velocity, val[slave]->torque, val[slave]->modeDisplay);
                     // printf("Error code %d\n", val[slave]->errorCode);
                     // printf("counter %d, position: %d, speed: %d, torque: %d, opMode: %d\n",
                     //        counter, target1->position, target1->speed, target1->torque, target1->opMode);
                  }
               }
            }

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
      }
   }
}

OSAL_THREAD_FUNC ecatcheck()
{
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
         for (int slave = 1; slave <= ec_slavecount; slave++)
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

OSAL_THREAD_FUNC keyboardcheck()
{
   printf("Enter an Operation Mode: [CSP:8,CSV:9,CST:10] ");
   while (1)
   {
      int testInteger;
      scanf("%d", &testInteger);
      printf("Operation Mode = %d -> ", testInteger);
      switch (testInteger)
      {
      case 1: // PP
         operation_mode = testInteger;
         printf("Profile Position Mode\n");
         break;
      case 3: // PV
         operation_mode = testInteger;
         printf("Profile Velocity Mode\n");
         break;
      case 4: // Torque
         operation_mode = testInteger;
         printf("Torque Mode\n");
         break;
      case 6: // Homing
         operation_mode = testInteger;
         printf("Homing Mode\n");
         break;
      case 7: // Interpolated position
         operation_mode = testInteger;
         printf("Interpolated Position Mode\n");
         break;
      case 8: // position
         operation_mode = testInteger;
         printf("Cyclic Sync Position Mode\n");
         break;
      case 9: // speed
         operation_mode = testInteger;
         printf("Cyclic Sync Velocity Mode\n");
         break;
      case 10: // torque
         operation_mode = testInteger;
         printf("Cyclic Sync Torque Mode\n");
         break;
      default:
         break;
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

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread3, stack64k * 6, &keyboardcheck, NULL);

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
