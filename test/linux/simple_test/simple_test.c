/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

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

void simpletest(char *ifname)
{
   int i, j, oloop, iloop, chk;
   needlf = FALSE;
   inOP = FALSE;
   uint16_t buf16;
   uint32_t buf32;
   uint8_t buf8;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

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

         printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 1000;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 1000);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;

            for (int i = 1; i <= ec_slavecount; i++)
            {
               READ(i, 0x1000, 0, buf32, "Unit Type");
               READ(i, 0x1001, 0, buf8, "Error register");

               printf("---------------------------------\n");
               READ(i, 0x1003, 0, buf32, "number of latest error");
               READ(i, 0x1003, 1, buf32, "latest error 1");
               READ(i, 0x1003, 2, buf32, "latest error 2");
               READ(i, 0x1003, 3, buf32, "latest error 3");
               READ(i, 0x1003, 4, buf32, "latest error 4");
               READ(i, 0x1003, 5, buf32, "latest error 5");
               READ(i, 0x1003, 6, buf32, "latest error 6");
               READ(i, 0x1003, 7, buf32, "latest error 7");
               READ(i, 0x1003, 8, buf32, "latest error 8");
               READ(i, 0x100B, 0, buf32, "node id set");
               printf("---------------------------------\n");

               READ(i, 0x10F1, 2, buf32, "Sync monitor Setup");

               READ(i, 0x1600, 0, buf32, "num of RPDO");
               READ(i, 0x1600, 1, buf32, "RPDO 1");
               READ(i, 0x1600, 2, buf32, "RPDO 2");
               READ(i, 0x1600, 3, buf32, "RPDO 3");
               READ(i, 0x1A00, 0, buf32, "num of TPDO");
               READ(i, 0x1A00, 1, buf32, "TPDO 1");
               READ(i, 0x1A00, 2, buf32, "TPDO 2");
               READ(i, 0x1A00, 3, buf32, "TPDO 3");
               READ(i, 0x1C00, 0, buf8, "SM Com Type");
               READ(i, 0x1C00, 1, buf8, "SM 1 Com Type");
               READ(i, 0x1C00, 2, buf8, "SM 2 Com Type");
               READ(i, 0x1C00, 3, buf8, "SM 3 Com Type");
               READ(i, 0x1C00, 4, buf8, "SM 4 Com Type");
               READ(i, 0x1C12, 0, buf8, "SM2");
               READ(i, 0x1C13, 0, buf8, "SM3");

               READ(i, 0x1C32, 0, buf32, "SM2 parameters");
               READ(i, 0x1C33, 0, buf32, "SM3 parameters");
               printf("---------------------------------\n");

               READ(i, 0x200A, 1, buf32, "motor bloccking current");
               READ(i, 0x200A, 2, buf32, "motor bloccking time");
               READ(i, 0x200A, 3, buf32, "motor bloccking speed"); //524
               // WRITE(i, 0x200A, 3, buf32, 5000, "motor bloccking speed");
               printf("---------------------------------\n");

               READ(i, 0x20A0, 0, buf32, "Additional Location");
               READ(i, 0x2205, 0, buf16, "Analog Input");
               READ(i, 0x2240, 0, buf32, "Motor End Encoder position [cnt]");
               READ(i, 0x2241, 0, buf32, "double encoder angle difference [cnt]");
               READ(i, 0x22A2, 0, buf16, "drive temperature [°C]");
               printf("---------------------------------\n");

               READ(i, 0x2380, 1, buf16, "Current ring gain");
               // WRITE(i, 0x2380, 1, buf16, 500, "Current ring gain");
               READ(i, 0x2380, 2, buf16, "Current ring integral");

               READ(i, 0x2381, 1, buf16, "Speed ring gain");
               READ(i, 0x2381, 2, buf16, "Speed ring integral");

               READ(i, 0x2382, 1, buf16, "Position ring gain");
               // READ(i, 0x2382, 2, buf16, "Position ring integral");

               READ(i, 0x3000, 0, buf32, "Upper limit of integral");
               READ(i, 0x3B60, 0, buf32, "Speed follow Error window [cnt /sec]");
               READ(i, 0x4602, 0, buf32, "Release brake");
               printf("---------------------------------\n");

               READ(i, 0x603F, 0, buf16, "Error Code");
               READ(i, 0x6040, 0, buf16, "Control Word");
               READ(i, 0x6041, 0, buf16, "Status Word");
               printf("---------------------------------\n");

               READ(i, 0x605A, 0, buf16, "Quick Stop Options code");
               READ(i, 0x605B, 0, buf16, "Close Options code");
               READ(i, 0x605C, 0, buf16, "Disable the Action Options code");
               READ(i, 0x605E, 0, buf16, "Failure response Options code");
               printf("---------------------------------\n");
               
               READ(i, 0x6060, 0, buf8, "Operation Mode");
               READ(i, 0x6061, 0, buf8, "Running Mode Display");
               READ(i, 0x6062, 0, buf32, "location requirement value");
               READ(i, 0x6063, 0, buf32, "actual location* [internal unit]");
               READ(i, 0x6064, 0, buf32, "actual location");
               READ(i, 0x6065, 0, buf32, "position follow error window"); //10484
               // WRITE(i, 0x6065, 0, buf32, 500, "position follow error window"); //
               READ(i, 0x6066, 0, buf16, "position follow error timeout");
               READ(i, 0x6067, 0, buf32, "position window");
               READ(i, 0x6068, 0, buf16, "position window time");
               printf("---------------------------------\n");

               READ(i, 0x6069, 0, buf32, "speed sensor actual value [cnt/sec]");
               READ(i, 0x606A, 0, buf16, "speed sensor selection code");
               READ(i, 0x606B, 0, buf32, "speed need value");
               READ(i, 0x606C, 0, buf32, "speed actual value [cnt/sec]");
               READ(i, 0x606D, 0, buf16, "speed window");
               READ(i, 0x606E, 0, buf16, "speed window time");
               READ(i, 0x606F, 0, buf16, "speed threshold");
               READ(i, 0x6070, 0, buf16, "speed threshold time");
               printf("---------------------------------\n");

               READ(i, 0x6071, 0, buf16, "Target torque");
               READ(i, 0x6072, 0, buf16, "Max Torque");
               READ(i, 0x6073, 0, buf16, "Max current");
               READ(i, 0x6074, 0, buf16, "torque Demand Value");
               READ(i, 0x6075, 0, buf32, "Motor rated current");
               READ(i, 0x6076, 0, buf32, "Motor rated torque");
               READ(i, 0x6077, 0, buf16, "Motor actual torque");
               READ(i, 0x6078, 0, buf16, "Motor actual current");
               printf("---------------------------------\n");
              
               READ(i, 0x6079, 0, buf32, "DC Link Voltage");
               printf("---------------------------------\n");

               READ(i, 0x607A, 0, buf32, "Target Location"); // ***** attenzione messaggio ---before enabling the motor!
               READ(i, 0x607B, 2, buf32, "Location Limit");
               READ(i, 0x607C, 0, buf32, "Origin Compensation");
               READ(i, 0x607D, 2, buf32, "Software location limit");
               READ(i, 0x607E, 0, buf8, "polarity");
               READ(i, 0x607F, 0, buf32, "maximum contour velocity");
               READ(i, 0x6080, 0, buf32, "maximum motor speed");
               READ(i, 0x6081, 0, buf32, "Contour speed");
               READ(i, 0x6082, 0, buf32, "End speed");
               READ(i, 0x6083, 0, buf32, "profile Acceleration");
               READ(i, 0x6084, 0, buf32, "Contour reduction speed");
               READ(i, 0x6085, 0, buf32, "rapid stop and decrease");
               READ(i, 0x6086, 0, buf16, "Sport profile type");
               printf("---------------------------------\n");

               READ(i, 0x6087, 0, buf32, "Torque slope");
               READ(i, 0x6093, 0, buf32, "Location factors");
               READ(i, 0x6094, 0, buf32, "Speed encoder coefficient");
               READ(i, 0x6095, 0, buf32, "Speed factor 1");
               READ(i, 0x6097, 0, buf32, "Acceleration factor");
               printf("---------------------------------\n");
               
               READ(i, 0x6098, 0, buf8, "Back to zero method");
               READ(i, 0x6099, 0, buf32, "Back to zero speed");
               READ(i, 0x609A, 0, buf32, "Back to zero acceleration");

               printf("---------------------------------\n");
               READ(i, 0x60B0, 0, buf32, "position offset value");
               READ(i, 0x60B1, 0, buf32, "speed offset value");
               READ(i, 0x60B2, 0, buf16, "torque offset value");

               printf("---------------------------------\n");
               READ(i, 0x60C0, 0, buf16, "pin mode selection");
               READ(i, 0x60C1, 0, buf32, "insert data records");
               READ(i, 0x60C2, 0, buf8, "interpolation time cycle");
               READ(i, 0x60C2, 1, buf8, "Interpolation Time Period Value");
               // WRITE(i, 0x60C2, 1, buf8, 1, "Set Interpolation Time Period Value");
               READ(i, 0x60C2, 2, buf8, "Interpolation Time Index");
               WRITE(i, 0x60C2, 2, buf8, 253, "Set Interpolation Time Index");
               READ(i, 0x60C5, 0, buf32, "maximal acceleration");
               READ(i, 0x60C6, 0, buf32, "maximum deceleration");

               printf("---------------------------------\n");
               READ(i, 0x60F2, 0, buf16, "location options code");
               READ(i, 0x60F4, 0, buf32, "Actual value of following error");
               READ(i, 0x60FC, 0, buf32, "Location demand value");
               READ(i, 0x60FD, 0, buf32, "Data input");
               READ(i, 0x60FE, 0, buf32, "Data output");
               READ(i, 0x60FF, 0, buf32, "target Speed");
               READ(i, 0x6502, 0, buf32, "Supported driver mode");

               printf("---------------------------------\n");
               // READ(i, 0x6088, 0, buf32, "TEST******");

               WRITE(i, 0x4602, 0, buf32, 1, "Release Brake");
               osal_usleep(100000);
               READ(i, 0x4602, 0, buf32, "Read Release Brake");
            }

            /* cyclic loop */
            for (i = 1; i <= 1; i++)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

               if (wkc >= expectedWKC)
               {
                  printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

                  for (j = 0; j < oloop; j++)
                  {
                     printf(" %2.2x", *(ec_slave[0].outputs + j));
                  }

                  printf(" I:");
                  for (j = 0; j < iloop; j++)
                  {
                     printf(" %2.2x", *(ec_slave[0].inputs + j));
                  }
                  printf(" T:%" PRId64 "\r", ec_DCtime);
                  needlf = TRUE;
               }
               osal_usleep(5000);
            }
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
         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
   }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

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

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      //      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
