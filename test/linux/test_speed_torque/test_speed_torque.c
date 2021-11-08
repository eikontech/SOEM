#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>

#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;


struct SpeedTorqueOut
{
    int32 position;
    int32 speed;
    int16 torque;
    int16 maxTorque;
    uint16 status;
    int8 operation;
};
struct SpeedTorqueIn
{
    uint16 error;
    uint16 status;
    int32 position;
    int32 speed;
    int16 torque;
    int8 operation;
};


/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

#define CHECKERROR(slaveId)                                                                                                                                                                       \
    {                                                                                                                                                                                             \
        ec_readstate();                                                                                                                                                                           \
        printf("EC> \"%s\" %x - %x [%s] \n", (char *)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
    }

void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

    struct SpeedTorqueIn *val1;
    struct SpeedTorqueOut *target1;

    printf("Starting speed test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x6060, 0, buf8, 9, "OpMode");     // Operation mode position
                READ(i, 0x6061, 0, buf8, "OpMode display");

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            int32 ob2;
            int os;
            for (int i = 1; i <= ec_slavecount; i++)
            {
                os = sizeof(ob2);
                ob2 = 0x16050001;
                ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                os = sizeof(ob2);
                ob2 = 0x1a060001;
                ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            /** if CA disable => automapping works */
            ec_config_map(&IOmap);

            // show slave info
            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                       ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }

            /** disable heartbeat alarm */
            for (int i = 1; i <= ec_slavecount; i++)
            {
                READ(i, 0x10F1, 2, buf32, "Heartbeat?");        // 0x10F1/2: SYNC Monitor setup - forse Sync Error counter limit, oltre il quale lo slave deve cabiare lo stato EtherCAT a SAFEOP.
                WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");     // lo slave deve resettare il contatore quando viene di nuovo sincronizzato.

                WRITE(i, 0x60c2, 1, buf8, 2, "Time period");        // Interpolation time cycle - 0: Number of entries, 1: Interpolation time period, 2: Interpolation time index 
                                                                    // cycle = period * (10^index) secondi. Mon sappiamo il default dell'index)
                WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");     // Extrapolation Cycles Timeout - 
            }

            printf("Slaves mapped, state to SAFE_OP.\n");

            int timestep = 700;

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 20)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 20)
                iloop = 8;

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
                READ(i, 0x6084, 0, buf32, "Profile deceleration");
                READ(i, 0x6076, 0, buf32, "Motor related torque");
                READ(i, 0x6080, 0, buf32, "Max Motor Speed");
                READ(i, 0x60c2, 0, buf32, "Interpolation time period");
            }

            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40    ;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    READ(i, 0x6041, 0, buf16, "*status word*");
                    if (buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*");
                        usleep(100000);
                        READ(i, 0x6041, 0, buf16, "*status word*");
                    }

                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    CHECKERROR(i);
                    READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    WRITE(i, 0x4602, 0, buf32, 1, "Release Brake");
                    usleep(100000);
                    READ(i, 0x4602, 0, buf32, "Read Release Brake");

                    READ(i, 0x1001, 0, buf8, "Error");
                }
                // int reachedInitial = 0;

                /* cyclic loop for two slaves*/
                target1 = (struct SpeedTorqueOut *)(ec_slave[1].outputs);
                val1 = (struct SpeedTorqueIn *)(ec_slave[1].inputs);

                for (i = 1; i <= 10000; i++)
                {
                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC)
                    {
                        printf("Processdata cycle %4d, WKC %d,", i, wkc);
                        printf("  err: 0x%x, status: 0x%x, pos: 0x%x, speed: 0x%x, torque: 0x%x, operation: 0x%x\n", val1->error, val1->status, val1->position, val1->speed, val1->torque, val1->operation);

                        if ((val1->status & 0b0000000001001111) == 0b0000000000000000) {         // Not ready to switch on
                            printf("Error: transition Not Ready to Switch On => Switch On Disabled should be automatic"); 
                        }
                        else if ((val1->status & 0b0000000001001111) == 0b0000000001000000) {  // Switch on disabled
                            target1->status = 6U;        // transition 2
                        }
                        else if ((val1->status & 0b0000000001101111) == 0b0000000000100001) {  // Ready to switch on
                            target1->status = 7U;        // transition 3
                        }
                        else if ((val1->status & 0b0000000001101111) == 0b0000000000100011) {  // Switched on
                            target1->status = 15U;       // transition 4
                        } 
                        else if ((val1->status & 0b0000000001001000) == 0b0000000000001000) {  // Fault + Fault reaction active
                            READ(1, 0x1001, 0, buf8, "Error");
                            target1->status = 128U;      // transition 15
                        }

                        if ((val1->status & 0b0000000001101111) == 0b0000000000100111 /*&& reachedInitial*/)        // Operation enabled
                        {
                            if (i<4000) {
                                target1->speed += i/10;
                            }
                            else if (i<8000) {
                                target1->speed -= i/10;
                            }
                            else {
                                target1->speed += i/10;
                            }
                        }
                        printf("  Target err: 0x%x, status: 0x%x, pos: 0x%x, speed: 0x%x, torque: 0x%x, operation: 0x%x\n", val1->error, val1->status, val1->position, val1->speed, val1->torque, val1->operation);
                        printf("  Val pos: 0x%x, speed: 0x%x, torque: 0x%x, maxTorque: 0x%x, status: 0x%x, operation: 0x%x\n", target1->position, target1->speed, target1->torque, target1->maxTorque, target1->status, target1->operation);

                        printf("\r");
                        needlf = TRUE;
                    }
                    usleep(timestep);
                }
                usleep(100000);
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
            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }

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

void *ecatcheck()
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
                    else if (ec_slave[slave].state > 0)
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
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
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
                printf(".");
        }
        usleep(250);
    }
}

int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime); // (void) &ctime
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program speed test\n");
    return (0);
}