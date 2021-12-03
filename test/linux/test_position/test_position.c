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
    int i, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    // uint16 buf16;
    uint8 buf8;

    struct PositionIn *val1;
    struct PositionOut *target1;

    printf("Starting speed test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {

            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("Has CA? %s\n", ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x6060, 0, buf8, 8, "OpMode"); // Operation mode position
                READ(i, 0x6061, 0, buf8, "OpMode display");
            }

            // int32 ob2;
            // int os;
            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x1c12, 0, buf8, 0, "disable RX");
                WRITE(i, 0x1c12, 1, buf32, 0x1603, "assign 1603");
                WRITE(i, 0x1c12, 0, buf8, 1, "enable RX");

                WRITE(i, 0x1c13, 0, buf8, 0, "disable TX");
                WRITE(i, 0x1c13, 1, buf32, 0x1a03, "assign 1A03");
                WRITE(i, 0x1c13, 0, buf8, 1, "enable TX");

                //     ob2 = 0x16000001;
                //     os = sizeof(ob2);
                //     ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                //     ob2 = 0x1a000001;
                //     os = sizeof(ob2);
                //     ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
            }

            ec_config_map(&IOmap);

            int timestep = 700;

            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            ec_writestate(0);
            chk = 40;

            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                inOP = TRUE;
                for (int i = 1; i <= ec_slavecount; i++) 
                {
                    WRITE(i, 0x4602, 0, buf32, 1, "Release Brake");
                    usleep(100000);
                    READ(i, 0x4602, 0, buf32, "Read Release Brake");

                    WRITE(i, 0x60b0, 0, buf32, 3000, "Position Offset");

                }

                /* cyclic loop for two slaves*/
                target1 = (struct PositionOut *)(ec_slave[1].outputs);
                val1 = (struct PositionIn *)(ec_slave[1].inputs);

                for (i = 1; i <= 10000; i++)
                {
                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC)
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
                            READ(1, 0x1001, 0, buf8, "Error");
                            READ(1, 0x1003, 0, buf32, "# of Error");
                            int max_err = buf32;
                            for (int j = 1; j <= max_err; ++j)
                            {
                                char label[8];
                                sprintf(label, "Error %hu", j);
                                READ(1, 0x1003, j, buf32, label);
                            }

                            target1->control = 128U; // transition 15
                        }

                        if ((val1->status & 0b0000000001101111) == 0b0000000000100111 /*&& reachedInitial*/) // Operation enabled
                        {
                            // printf("set position.\n");
                            target1->position = 2000000;
                            target1->velocity_offset = 500;
                        }
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
            }
            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
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
            // if (needlf)
            // {
            //     needlf = FALSE;
            //     printf("\n");
            // }
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
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
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
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
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
    return (0);
}
