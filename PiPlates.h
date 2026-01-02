// Header for using the pi plates. It's not as complete as it could be
// but its a good start. It relies on the pigpio.h
// at the start of the code gpioInitialise(); and PiPlateInitialize() function must be called to
// set up the spi, then the functions are very similar to they used in the python code
#define GPIObaseADDR 8
#define RELAYbaseADDR 24
#define MOTORbaseADDR 16
#define RMAX 2000
#define ppFRAME 25
#define ppINT 22
#define PiPlatesComLine 1
#define PiPlatesComFreq 250000


#pragma once
#include <stdio.h>          /// IO functions

#pragma once
#include <stdbool.h>        /// Boolean type and values

#pragma once
#include <unistd.h>         /// Used to fork and pipe

#pragma once
#include <ctype.h>         /// used for tolower()

#pragma once
#include <pigpio.h>

volatile double PiPlatesDACVcc = 0;
volatile int HANDLE = 0;

//////////////////////////////////
/// Basic Pi Plates Functions ///
//////////////////////////////////

/// Initializes the pi plate by setting up the SPI com ///
void PiPlateInitialize()
{
    gpioSetMode(ppFRAME, PI_OUTPUT);
    gpioSetMode(ppINT, PI_INPUT);
    gpioSetPullUpDown(ppINT, PI_PUD_UP);
    HANDLE = spiOpen(PiPlatesComLine, PiPlatesComFreq, 0);
}

/// The way all the DAC functions talk to the DAC pi plates ///
void ppCMD(char addr, char cmd, char param1, char param2, char bytes2return, char *resp)
{
    char arg[4], i;
    arg[0] = addr + GPIObaseADDR;
    arg[1] = cmd;
    arg[2] = param1;
    arg[3] = param2;

    gpioWrite(ppFRAME, 1); //digitalWrite(ppFRAME, HIGH);
    spiWrite(HANDLE, arg, 4); //wiringPiSPIDataRW(PiPlatesComLine, arg, 4);
    usleep(1);
    if(bytes2return>0)
    {
        usleep(100);
        for(i=0;i<bytes2return;++i)
        {
            spiRead(HANDLE, &resp[i], 1); //wiringPiSPIDataRW(PiPlatesComLine, &resp[i], 1);
            usleep(1);
        }
    }
    gpioWrite(ppFRAME, 0); //digitalWrite(ppFRAME, LOW);

    return;
}

/// The way all the Relay functions talk to the relay pi plates ///
void ppCMDr(char addr, char cmd, char param1, char param2, char bytes2return, char *resp)
{
    char arg[4], i;
    arg[0] = addr + RELAYbaseADDR;
    arg[1] = cmd;
    arg[2] = param1;
    arg[3] = param2;

    gpioWrite(ppFRAME, 1);
    spiWrite(HANDLE, arg, 4);
    if(bytes2return>0)
    {
        usleep(100);
        for(i=0;i<bytes2return;++i)
            spiRead(HANDLE, &resp[i], 1);
    }
    usleep(1000);
    gpioWrite(ppFRAME, 0);
    usleep(1000);

    return;
}

/// The way all the motor functions talk to the motor plates ///
void ppCMDm(char addr, char cmd, char param1, char param2, char bytes2return, char *resp)
{
    char arg[4], i;
    arg[0] = addr + MOTORbaseADDR;
    arg[1] = cmd;
    arg[2] = param1;
    arg[3] = param2;

    gpioWrite(ppFRAME, 1); //digitalWrite(ppFRAME, HIGH);
    spiWrite(HANDLE, arg, 4); //    wiringPiSPIDataRW(PiPlatesComLine, arg, 4);
    if(bytes2return>0)
    {
        usleep(100);
        for(i=0;i<bytes2return;++i)
        {
            spiRead(HANDLE, &resp[i], 1);  //            wiringPiSPIDataRW(PiPlatesComLine, &resp[i], 1);
            usleep(1);
        }
    }
    usleep(2000);
    gpioWrite(ppFRAME, 0); //digitalWrite(ppFRAME, LOW);

    return;
}

/////////////////////
/// DAC Functions ///
/////////////////////

/// ADC Functions ///
// Return voltage from single channel. Reading channel 8 will return the 5VDC power supply voltage. //
double PiPlatesDAC_getADC(char addr, char channel)
{
    char resp[2] = {0};

    ppCMD(addr, 0x30, channel, 0, 2, resp);

    return (256.0*resp[0] + resp[1])*4.096/1024;
}

// Puts voltage from all channels into Val //
void PiPlatesDAC_getADCall(char addr, double Val[8])
{
    char resp[16] = {0}, i;

    ppCMD(addr, 0x31, 0, 0, 16, resp);

    for(i=0;i<8;++i)
        Val[i] = (256.0*resp[2*i] + resp[2*i+1])*4.096/1024.0;
}

/// Digital Input Functions (missing interrupt functions) ///
// Returns single bit value //
bool PiPlatesDAC_getDINbit(char addr, char bit)
{
    char resp = 0;
    ppCMD(addr, 0x20, bit, 0, 1, &resp);
    usleep(1);
    return resp == 1 ? true : false;
}

// Returns all eight bits //
char PiPlatesDAC_getDINall(char addr)
{
    char resp = 0;
    ppCMD(addr, 0x25, 0, 0, 1, &resp);
    usleep(1);
    return resp;
}

/// LED functions ///
// Turn on one of the LEDs in the bicolor LED package (0 - red, 1 - green) //
#define PiPlatesDAC_setLED(addr, led) ppCMD(addr, 0x60, led, 0, 0, NULL); usleep(1);

// Turn off one of the LEDs in the bicolor LED package //
#define PiPlatesDAC_clrLED(addr, led) ppCMD(addr, 0x61, led, 0, 0, NULL); usleep(1);

// Toggle off one of the LEDs in the bicolor LED package //
#define PiPlatesDAC_toggleLED(addr, led) ppCMD(addr, 0x62, led, 0, 0, NULL); usleep(1);

// Returns the status of the LEDs in the bicolor LED package //
char PiPlatesDAC_getLED(int addr, int led)
{
    char resp = 0;
    ppCMD(addr, 0x63, led, 0, 1, &resp);
    return resp;
}

/// Switch Functions WIP///

/// Digital Output Functions ///
// Set single bit //
#define PiPlatesDAC_setDOUTbit(addr, bit) ppCMD(addr, 0x10, bit, 0, 0, NULL); usleep(100)

// Clear single bit ///
#define PiPlatesDAC_clrDOUTbit(addr, bit) ppCMD(addr, 0x11, bit, 0, 0, NULL); usleep(100)

// Toggle a single bit //
#define PiPlatesDAC_toggleDOUTbit(addr, bit) ppCMD(addr, 0x12, bit, 0, 0, NULL); usleep(100)

// Control all seven bits at once. The byte value must be in the range of 0 through 127 ///
#define PiPlatesDAC_setDOUTall(addr, bit) ppCMD(addr, 0x13, bit, 0, 0, NULL); usleep(100)

// Return the status of the ouput bit //
char PiPlatesDAC_getDOUTbyte(char addr)
{
    char resp = 0;
    ppCMD(addr, 0x14, 0, 0, 1, &resp);
    usleep(1);
    return resp;
}

/// PWM and DAC Output Functions ///
// Set PWM signal from 0 to 1023 (0 to 100%) //
void PiPlatesDAC_setPWM(char addr, char channel, short Val)
{
    char hibyte, lobyte;
	hibyte = Val>>8;
    lobyte = Val - (hibyte<<8);

    ppCMD(addr, 0x40+channel, hibyte, lobyte, 0, NULL);

    usleep(100);
}

// Return current PWM setting. Returned value will be a number between 0 and 1023 ///
short PiPlatesDAC_getPWM(char addr, char channel)
{
    short Val;
    char resp[2] = {0};

    ppCMD(addr, 0x40+channel+2, 0, 0, 2, resp);

    Val = 256*resp[0] + resp[1];
    return Val;
}

// Set DAC output voltage from 0 to 4.097 volts //
// Needs about 10ms to settle
void PiPlatesDAC_setDAC(char addr, char channel, double Val)
{
    short IntVal;
    char hibyte, lobyte;

    if(PiPlatesDACVcc == 0)
    {
        printf("Need to run the PiPlatesDAC_calDAC function first");
        return;
    }

    IntVal = (short)(Val/PiPlatesDACVcc*1024);
	hibyte = IntVal>>8;
    lobyte = IntVal - (hibyte<<8);

    ppCMD(addr, 0x40+channel, hibyte, lobyte, 0, NULL);

    usleep(10);

    return;
}

// Cal the analog output //
void PiPlatesDAC_calDAC(char addr)
{
    PiPlatesDACVcc = PiPlatesDAC_getADC(addr, 8) * 2.0;
    sleep(1);
}



/////////////////////////////
/// Relay Board functions ///
/////////////////////////////


/// Relay functions ///
// Turns on (closes) the specified relay //
#define RelayBoard_relayON(addr, relay) ppCMDr(addr, 0x10, relay, 0, 0, NULL)

// Turns off (opens) the specified relay //
#define RelayBoard_relayOFF(addr, relay) ppCMDr(addr, 0x11, relay, 0, 0, NULL)

// Toggles state of specified relay. If relay is on, this command will turn it off
// If relay is off, this command will turn it on
#define RelayBoard_relayTOGGLE(addr, relay) ppCMDr(addr, 0x12, relay, 0, 0, NULL)

// Controls the state of all relays with a single command “value” is a 7 bit number
// with each bit corresponding to a relay. Bit 0 is relay 1, bit 1 is relay 2, and so on
#define RelayBoard_relayALL(addr, relay) ppCMDr(addr, 0x13, relay, 0, 0, NULL)

// Returns a 7-bit number with the current state of each relay. Bit 0 is relay 1, bit 1 is relay 2, and so on
// 1 - relay is on, zero - relay is off
char RelayBoard_relaySTATE(char addr)
{
    char resp = 0;
    ppCMDr(addr, 0x14, 0, 0, 1, &resp);

    return resp;
}

/// LED functions ///
// turn on the LED //
#define RelayBoard_setLED(addr) ppCMDr(addr, 0x60, 0, 0, 0, NULL)

// turn off the LED //
#define RelayBoard_clrLED(addr) ppCMDr(addr, 0x61, 0, 0, 0, NULL)

// Toggle the LED //
#define RelayBoard_toggleLED(addr) ppCMDr(addr, 0x62, 0, 0, 0, NULL)


/////////////////////////////
/// Motor Board functions ///
/////////////////////////////

// Little function needed in stepperCONFIG //
char parseRES(char res)
{
    switch(res)
    {
        case 'f':
        case '1':
            return 0;
        case 'h':
        case '2':
            return 1;
        case '4':
            return 2;
        case '8':
            return 3;

        default:
            return -1;
    }

    return -1;
}

// Configures the motor dir: '+' or '-', resolution 'f', 'h', '4', or '8' //
void stepperCONFIG(char addr, char motor, char dir, char resolution, short rate, char acceleration)
{
    char param1, param2;
    int increment;

    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }

    if((dir!='+') && (dir!='-'))
    {
        printf("ERROR: incorrect direction parameter '+' or '-'\n");
        return;
    }

    resolution = parseRES(resolution);
    if(resolution == -1)
    {
        printf("ERROR: incorrect resolution value\n");
        return;
    }

    if((rate>RMAX) || (rate<1))
    {
        printf("ERROR: incorrect rate value\n");
        return;
    }

    if((acceleration>10) || (acceleration<0))
    {
        printf("ERROR: incorrect acceleration time 1-10\n");
        return;
    }

    // Assemble Param1:|0|DIR|RES1|RES2|NA|RATE10|RATE9|RATE8|
    param1 = (dir == '-') ? 0x40 : 0;
    param1 += resolution<<4;
    param1 += rate>>8;
    // Assemble Param2:|RATE7-RATE0|
    param2 = rate & 0x00FF;
    ppCMDm(addr,(motor=='b') ? 0x11 : 0x10,param1,param2,0,NULL);
    usleep(10000);      //Allow uP on board time to process
    // Send 2nd set of parameters but with same command numbers to add acceleration
    // increment. This is a 15 bit number with the upper5 bits being the integer value
    // and the lower 10 bits being the fractional part
    increment = (acceleration==0) ? 0 : (int)(1024*rate/(acceleration*RMAX)+0.5);
    // Param1:|1|ACC14|ACC13|ACC12|ACC11|ACC10|ACC9|ACC10|
    param1 = 0x80 + (increment>>8);
    param2 = increment & 0x00FF;

    ppCMDm(addr,(motor=='b') ? 0x11 : 0x10, param1, param2, 0, NULL);

    usleep(400000);      // Allow board time to process

    return;
}

// Moves motor steps as defined by stepperCONFIG //
void stepperMOVE(char addr, char motor, unsigned short steps)
{
    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }

    ppCMDm(addr, (motor=='b') ? 0x13 : 0x12, steps>>8, steps&0xFF, 0, NULL);
}

// starts motor as defined by stepperCONFIG //
void stepperJOG(char addr, char motor)
{
    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }

    ppCMDm(addr, (motor=='b') ? 0x15 : 0x14, 0, 0, 0, NULL);

    return;
}

// Stops the motor when jogging //
void stepperSTOP(char addr, char motor)
{
    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }

    ppCMDm(addr, (motor=='b') ? 0x17 : 0x16, 0, 0, 0, NULL);

    usleep(100000);

    return;
}

// Changed the rate can be done mid jog //
void stepperRATE(char addr, char motor, short rate)
{
    char param1, param2;

    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }
    if((rate>RMAX) || (rate<1))
    {
        printf("ERROR: incorrect rate value\n");
        return;
    }
    // Assemble Param1:|0|0|0|0|0|RATE10|RATE9|RATE8|
    param1 = (rate>>8);
    // Assemble Param2:|RATE7-RATE0|
    param2 = rate & 0xFF;
    ppCMDm(addr, (motor=='b') ? 0x19 : 0x18, param1, param2, 0, NULL);

    return;
}

// Shuts off power to motor //
void stepperOFF(char addr, char motor)
{
    motor = tolower(motor);
    if((motor!='a') && (motor!='b'))
    {
        printf("ERROR: incorrect stepper motor selection 'a' or 'b'\n");
        return;
    }

    ppCMDm(addr, (motor=='b') ? 0x1F : 0x1E, 0, 0, 0, NULL);

    return;
}

// Read INT flag register0 in MOTORplate //
char getINTflag0(char addr)
{
    char resp;
    ppCMDm(addr,0x06,0,0,1, &resp);
    return resp;
}


/// LED functions ///
// Set the motor plate LED //
#define PiPlatesMotor_setLED(addr) ppCMDm(addr, 0x60, 0, 0, 0, NULL)

// Turn off the led //
#define PiPlatesMotor_clrLED(addr) ppCMDm(addr, 0x61, 0, 0, 0, NULL)

// Toggle the led //
#define PiPlatesMotor_toggleLED(addr) ppCMDm(addr, 0x62, 0, 0, 0, NULL)