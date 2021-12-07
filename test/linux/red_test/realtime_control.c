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

typedef struct
{
   int32 position;
   int32 velocity;
   int16 torque;
   uint16 maxTorque;
   uint16 control;
   uint8 opMode;
   // uint8 padding;
   // uint16 torqueOffset;
} ControlOut;

typedef struct
{
   uint16 errorCode;
   uint16 status;
   int32 position;
   int32 velocity;
   int16 torque;
   uint8 modeDisplay;
   // uint8 padding;
   int16 current;
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

int operation_mode = 9;
#define INITIAL_POS 0
#define INITIAL_VEL 0
#define INITIAL_TOR 0

ControlIn *val[EC_MAXSLAVE];
ControlOut *target[EC_MAXSLAVE];
int max_torque = 10;
int max_current = 10;

int normal_velocity = 200;
int max_velocity = 5000;

int location_limit = 200000;

int default_position = INITIAL_POS;
int default_velocity = INITIAL_VEL;
int default_torque = INITIAL_TOR;
int target_position = INITIAL_POS;
int target_velocity = INITIAL_VEL;
int target_torque = INITIAL_TOR;

int motor_rated_current_mA = 0;
int motor_rated_torque_mNm = 0;

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
         }

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            // WRITE(slave, 0x6081, 0, buf32, 100000, "Set Contour Speed");
            // WRITE(slave, 0x6083, 0, buf32, 5566, "Set Profile Acceleration");
            // WRITE(slave, 0x6084, 0, buf32, 5566, "Set profile Deceleration");

            // READ(slave, 0x6081, 0, buf32, "Read Contour Speed");
            // READ(slave, 0x6083, 0, buf32, "Read Profile Acceleration");
            // READ(slave, 0x6084, 0, buf32, "Read profile Deceleration");

            READ(slave, 0x6064, 0, buf32, "Read Actual Position");
            osal_usleep(EC_TIMEOUTTXM);
            int32_t actual_location = buf32;
            char label[60];
            sprintf(label, "Set Actual Position %d", actual_location);
            WRITE(slave, 0x607A, 0, buf32, actual_location, label);
            // target_position = actual_location + 10000;
            osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x3B60, 0, buf32, 100000, "Write Speed Follow Error Window");
            WRITE(slave, 0x6065, 0, buf32, 50000, "Write Position Follow Error Window");

            READ(slave, 0x3000, 0, buf32, "Read Speed Loop integral upper limit");
            osal_usleep(EC_TIMEOUTTXM);
            // WRITE(slave, 0x3000, 0, buf32, 80000, "Set Speed Loop integral upper limit");
            // osal_usleep(EC_TIMEOUTTXM);
            // READ(slave, 0x3000, 0, buf32, "Read Speed Loop integral upper limit");

            READ(slave, 0x607d, 0, buf32, "Read Location Limit/0");
            READ(slave, 0x607d, 1, buf32, "Read Location Limit/1");
            READ(slave, 0x607d, 2, buf32, "Read Location Limit/2");
            location_limit = buf32;

            // WRITE(slave, 0x607f, 0, buf32, 314572, "Set Maximum Velocity");
            // osal_usleep(EC_TIMEOUTTXM);

            // WRITE(slave, 0x6080, 0, buf32, 314572, "Set Maximum Motor Velocity");
            READ(slave, 0x6080, 0, buf32, "Set Maximum Motor Velocity");
            // osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6081, 0, buf32, 50000, "Set Profile Velocity");
            osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6083, 0, buf32, 262144, "Set Profile Aceleration");
            osal_usleep(EC_TIMEOUTTXM);

            WRITE(slave, 0x6084, 0, buf32, 262144, "Set Profile Deceleration");
            osal_usleep(EC_TIMEOUTTXM);

            // WRITE(slave, 0x6075, 0, buf32, 10000, "Set Motor Rated Current [mA]");
            READ(slave, 0x6075, 0, buf32, "Read Motor Rated Current [mA]");
            motor_rated_current_mA = buf32 / 1000;
            // WRITE(slave, 0x6076, 0, buf32, 10000, "Set Motor Rated Torque [mNm]");
            READ(slave, 0x6076, 0, buf32, "Read Motor Rated Torque [mNm]");
            motor_rated_torque_mNm = buf32 / 1000;
            // osal_usleep(EC_TIMEOUTTXM);

            READ(slave, 0x6081, 0, buf32, "Read Profile Velocity");
            normal_velocity = buf32;
            READ(slave, 0x607f, 0, buf32, "Read Maximum Velocity");
            max_velocity = buf32;

            READ(slave, 0x607A, 0, buf32, "Read Target Location");
            READ(slave, 0x6064, 0, buf32, "Read Actual Location");
            READ(slave, 0x6041, 0, buf8, "Read Status Word");

            WRITE(slave, 0x6072, 0, buf16, 1833, "Write Max Torque");
            READ(slave, 0x6072, 0, buf16, "Read Max Torque");
            max_torque = buf16 * motor_rated_current_mA;

            WRITE(slave, 0x6073, 0, buf16, 1833, "Write Max Current");
            READ(slave, 0x6073, 0, buf16, "Read Max Current");
            max_current = buf16 * motor_rated_current_mA;

            // WRITE(slave, 0x60B2, 0, buf16, 200, "Write Torque offset");
            // WRITE(slave, 0x60B2, 0, buf16, 0, "Write Torque offset");

            // WRITE(slave, 0x200A, 3, buf32, 50000, "Write Motor Blocking Speed");

         }

         for (int slave = 1; slave <= ec_slavecount; slave++)
         {
            // RX - target
            WRITE(slave, 0x1c12, 0, buf8, 0, "disable RX");
            WRITE(slave, 0x1c12, 1, buf16, 0x1605, "assign 1605");
            // WRITE(slave, 0x1c12, 1, buf16, 0x1618, "assign 1618");
            WRITE(slave, 0x1c12, 0, buf8, 1, "enable RX");

            // TX - val
            WRITE(slave, 0x1c13, 0, buf8, 0, "disable TX");
            WRITE(slave, 0x1c13, 1, buf32, 0x1a06, "assign 1A06");
            WRITE(slave, 0x1c13, 2, buf32, 0x1a1f, "assign 1A1F");
            WRITE(slave, 0x1c13, 0, buf8, 2, "enable TX");
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

double error_cumulative = 0;
double kp = 0.5;
double ki = 0.01;
int blink = 0;

int window = 250;
double **values_position = NULL;
double **values_velocity = NULL;
double **values_torque = NULL;
double **values_current = NULL;

double *med_position = 0;
double *med_velocity = 0;
double *med_torque = 0;
double *med_current = 0;

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   /* File pointer to hold reference of input file */
   FILE *fPtr_med;
   FILE *fPtr_curr;
   /* Input file path to remove empty lines from user */
   char *filePath_med = "./out_med.txt";
   char *filePath_curr = "./out_curr.txt";

   /*  Open all file in append mode. */
   fPtr_med = fopen(filePath_med, "w");
   fPtr_curr = fopen(filePath_curr, "w");

   /* fopen() return NULL if unable to open file in given mode. */
   if (fPtr_med == NULL)
   {
      /* Unable to open file hence exit */
      printf("\nUnable to open '%s' file.\n", filePath_med);
      printf("Please check whether file exists and you have write privilege.\n");
      exit(EXIT_FAILURE);
   }

   if (fPtr_curr == NULL)
   {
      /* Unable to open file hence exit */
      printf("\nUnable to open '%s' file.\n", filePath_curr);
      printf("Please check whether file exists and you have write privilege.\n");
      exit(EXIT_FAILURE);
   }

   struct timespec ts, tleft;
   int ht;
   int64 cycletime;
   boolean *plotted;
   boolean initialized = FALSE;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = *(int *)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   uint counter = 0;

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
            if (!initialized)
            {
               values_position = calloc(ec_slavecount, sizeof(double *));
               values_velocity = calloc(ec_slavecount, sizeof(double *));
               values_torque = calloc(ec_slavecount, sizeof(double *));
               values_current = calloc(ec_slavecount, sizeof(double *));

               med_position = calloc(ec_slavecount, sizeof(double));
               med_velocity = calloc(ec_slavecount, sizeof(double));
               med_torque = calloc(ec_slavecount, sizeof(double));
               med_current = calloc(ec_slavecount, sizeof(double));

               plotted = calloc(ec_slavecount, sizeof(boolean));

               for (int slave = 1; slave <= ec_slavecount; slave++)
               {
                  values_position[slave] = calloc(window, sizeof(double));
                  values_velocity[slave] = calloc(window, sizeof(double));
                  values_torque[slave] = calloc(window, sizeof(double));
                  values_current[slave] = calloc(window, sizeof(double));
               }

               for (int slave = 1; slave <= ec_slavecount; slave++)
               {
                  for (int i = 0; i < window; ++i) // Set the first 6 elements in the array
                  {
                     values_position[slave][i] = 0;
                     values_velocity[slave][i] = 0;
                     values_torque[slave][i] = 0;
                     values_current[slave][i] = 0;
                  }
               }
               initialized = TRUE;
            }

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
                        sprintf(label, "Error %hu", j);
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
                     target[slave]->position = target_position;
                     target[slave]->position *= blink ? -1 : 1;

                     if (operation_mode == 8)
                     {
                        if(1)
                        {
                           double error = target[slave]->position - val[slave]->position;
                           if (fabs(error) < 50)
                           {
                              error_cumulative = 0;
                              target[slave]->velocity = 0;
                           }
                           else
                           {
                              if (fabs(error_cumulative) > 100000)
                                 error_cumulative = 0;

                              target[slave]->velocity = kp * error + ki * error_cumulative;
                              // printf("error: %.0f, cumulative: %.0f\n", error, error_cumulative);
                              // printf("target[slave]->velocity: %d\n", target[slave]->velocity);
                              error_cumulative += error;
                              // target[slave]->velocity = 0;
                              if (abs(target[slave]->velocity) > normal_velocity)
                                 target[slave]->velocity = target[slave]->velocity > 0 ? normal_velocity : -normal_velocity;
                           }
                        }
                     }
                     else if (operation_mode == 9)
                     {
                        target[slave]->velocity = target_velocity;
                        target[slave]->velocity *= blink ? -1 : 1;
                     }
                     else if (operation_mode == 10)
                     {
                        if (1)
                        {
                           double local_target_torque = target_torque;
                           local_target_torque *= (blink ? -1 : 1);
                           // target[slave]->velocity = max_velocity;

                           double error = local_target_torque - val[slave]->torque;
                           // double error = local_target_torque - med_torque;
                           // if (fabs(error) < 5)
                           // {
                           //    printf("OCCHIO in target torque! error %.0f\n", error);
                           //    error_cumulative = 0;
                           //    target[slave]->torque = 0;
                           // }
                           // else
                           {
                              if (fabs(error_cumulative) > 1000)
                                 error_cumulative = 0;

                              target[slave]->torque = kp * error + ki * error_cumulative;

                              // printf("error: %.0f, cumulative: %.0f\n", error, error_cumulative);
                              // printf("Target torque not limited %d\n", target[slave]->torque);

                              error_cumulative += error;
                              // int compare_torque = target[slave]->torque > 32767 ? target[slave]->torque - 65535: target[slave]->torque;
                              if (abs(target[slave]->torque) > 900)
                                 target[slave]->torque = (target[slave]->torque) > 0 ? 900 : -900;

                              // printf("Target torque limited %d\n", target[slave]->torque);
                           }
                        }
                        else
                        {
                           target[slave]->velocity = max_velocity;
                           target[slave]->torque = target_torque;
                           target[slave]->torque *= blink ? -1 : 1;
                           // target[slave]->torqueOffset = -9075;
                        }
                     }

                     if (abs(val[slave]->position) > location_limit && operation_mode != 8)
                     {
                        target[slave]->velocity = 0;
                        target[slave]->torque = 0;
                        target_position = target[slave]->position > 0 ? location_limit : -location_limit;
                        target_position = target_position / 2;
                        target[slave]->opMode = 9;
                        operation_mode = 8;
                     }

                     {
                        // double current_current = val[slave]->current > 32767? val[slave]->current - 65535 :val[slave]->current;
                        double current_current = val[slave]->current;

                        med_position[slave] = med_position[slave] + 1. / window * (val[slave]->position - values_position[slave][0]);
                        med_velocity[slave] = med_velocity[slave] + 1. / window * (val[slave]->velocity - values_velocity[slave][0]);
                        med_torque[slave] = med_torque[slave] + 1. / window * (val[slave]->torque - values_torque[slave][0]);
                        med_current[slave] = med_current[slave] + 1. / window * (current_current - values_current[slave][0]);

                        /* Append data to file */
                        char str[80];
                        sprintf(str, "%d %d %.0f %.0f %.0f\n", slave, counter, med_torque[slave] * motor_rated_current_mA, med_current[slave] * motor_rated_current_mA, med_velocity[slave]);
                        fputs(str, fPtr_med);
                        sprintf(str, "%d %d %d %.f %d\n", slave, counter, val[slave]->torque * motor_rated_current_mA, current_current * motor_rated_current_mA, val[slave]->velocity);
                        fputs(str, fPtr_curr);

                        fflush(fPtr_med);  // To clear extra white space characters in file
                        fflush(fPtr_curr); // To clear extra white space characters in file

                        for (int i = 0; i < window - 1; ++i)
                        {
                           values_position[slave][i] = values_position[slave][i + 1];
                           values_velocity[slave][i] = values_velocity[slave][i + 1];
                           values_torque[slave][i] = values_torque[slave][i + 1];
                           values_current[slave][i] = values_current[slave][i + 1];
                        }

                        values_position[slave][window - 1] = (double)val[slave]->position;
                        values_velocity[slave][window - 1] = (double)val[slave]->velocity;
                        values_torque[slave][window - 1] = (double)val[slave]->torque;
                        values_current[slave][window - 1] = current_current;
                     }

                     if (((fabs(med_position[slave] - val[slave]->position) < 200 && operation_mode == 8) ||
                        (fabs(med_velocity[slave] - val[slave]->velocity) < 3000 && operation_mode == 9) ||
                        (fabs(med_torque[slave] - val[slave]->torque) * motor_rated_current_mA < 1000 && operation_mode == 10)))
                     {
                        if (!plotted[slave])
                        {
                           plotted[slave] = TRUE;
                           printf("********** Slave %d Objective Reached at %d **************\n", slave, counter);
                        }
                     }
                     else
                     {
                        // printf("********** Slave %d Objective NOT Reached at %d **************\n", slave, counter);
                        plotted[slave] = FALSE;
                     }

                     if (counter % 10000 == 0 || 0)
                     {
                        printf("------------------------------------------------------------------------------\n");
                        // blink = !blink;
                        printf("Slave %d\n", slave);
                        printf("counter %d\n", counter);
                        printf("target:\tposition:\t%d,\t\tspeed:\t%d,\ttorque:\t%d,\topMode:\t%d,\tcurrent:\t%d\n",
                               target[slave]->position, target[slave]->velocity, target_torque * motor_rated_current_mA * (blink ? -1 : 1), target[slave]->opMode, target[slave]->torque * motor_rated_current_mA);
                        printf("actual:\tposition:\t%d,\tspeed:\t%d,\ttorque:\t%d,\topMode:\t%d,\tcurrent:\t%d\n",
                               val[slave]->position, val[slave]->velocity, val[slave]->torque * motor_rated_current_mA,
                               val[slave]->modeDisplay,
                               val[slave]->current * motor_rated_current_mA); //val[slave]->current > 32767? val[slave]->current - 65535 :val[slave]->current);
                        printf("Medium:\tposition:\t%.0f,\tspeed:\t%.0f,\ttorque:\t%.0f,\topMode:\t%d,\tcurrent:\t%.0f\n",
                               med_position[slave], med_velocity[slave], med_torque[slave] * motor_rated_current_mA,
                               val[slave]->modeDisplay,
                               med_current[slave] * motor_rated_current_mA);
                        printf("Error code %d\n", val[slave]->errorCode);
                        // printf("************************************************************************\n");
                     }
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

   free(values_position); // Un-reserve the second array
   free(values_velocity); // Un-reserve the second array
   free(values_torque);   // Un-reserve the second array
   free(values_current);  // Un-reserve the second array

   /* Done with file, hence close file. */
   fclose(fPtr_curr);
   fclose(fPtr_med);
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
      int error = scanf("%d", &testInteger);
      if(error == 0)
         continue;
      printf("Operation Mode = %d -> ", testInteger);
      switch (testInteger)
      {
      case 1: // PP
         operation_mode = testInteger;
         blink = !blink;
         printf("Profile Position Mode\n");
         break;
      case 3: // PV
         operation_mode = testInteger;
         blink = !blink;
         printf("Profile Velocity Mode\n");
         break;
      case 4: // Torque
         operation_mode = testInteger;
         blink = !blink;
         printf("Torque Mode\n");
         break;
      case 6: // Homing
         operation_mode = testInteger;
         blink = !blink;
         printf("Homing Mode\n");
         break;
      case 7: // Interpolated position
         operation_mode = testInteger;
         blink = !blink;
         printf("Interpolated Position Mode\n");
         break;
      case 8: // position
         operation_mode = testInteger;
         blink = !blink;
         target_position = target_position + 1000;
         target_velocity = default_velocity;
         target_torque = default_torque;
         error_cumulative = 0;
         printf("Cyclic Sync Position Mode\n");
         break;
      case 9: // speed
         operation_mode = testInteger;
         blink = !blink;
         target_position = default_position;
         target_velocity = target_velocity + 5000;
         target_torque = default_torque;
         error_cumulative = 0;
         printf("Cyclic Sync Velocity Mode\n");
         break;
      case 10: // torque
         operation_mode = testInteger;
         blink = !blink;
         target_position = default_position;
         target_velocity = default_velocity;
         target_torque = target_torque + 100;
         error_cumulative = 0;
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

   osal_usleep(1000000);

   return (0);
}
