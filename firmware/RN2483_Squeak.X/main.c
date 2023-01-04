/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 3.16
        Device            :  PIC18LF46K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.35
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/*
 * Project modified by M. Cuciuc for the Squeak GPS tracker
 *  - removed RN command parser
 *  - added NMEA parser for GPS positioning
 *  - added temperature & battery voltage measurement
 *  - added LORAWAN_CanSend() function to LoRaWAN library
 *  - added Squeak application logic, including downlink processing
 */


#include <pic18lf46k22.h>

#include "mcc_generated_files/mcc.h"

#include "system/system.h"
#include "sw_timer.h"
#include "system/system_low_power.h"

#include "radio_interface.h"
#include "radio_driver_hal.h"

#include "lorawan.h"
#include "lorawan_eu.h"
#include "lorawan_private.h"

#include "gps.h"


// ------------- Application state and GPS state definitions --------------

#define STATE_APP_NOT_INIT          0
#define STATE_APP_NOT_JOINED        1
#define STATE_APP_SLEEP_JOIN        2
#define STATE_APP_JOINING           3
#define STATE_APP_GO_SLEEP          4
#define STATE_APP_SLEEP             5
#define STATE_APP_WAKE              6
#define STATE_APP_GPS               7
#define STATE_APP_GPS_OFF           8
#define STATE_APP_TX                9
#define STATE_APP_TX_WAIT           10

#define STATE_GPS_OFF               0
#define STATE_GPS_ON_FOR            1
#define STATE_GPS_ON_EVERY          2


// ---------------------------- LoRaWAN credentials -----------------------

// devEUI is read from the RN2483 as it contains a unique serial number
uint8_t appEUI[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};        // aka joinEUI
uint8_t appKey[] = {0xDE, 0xAD, 0xC0, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ------------ Global variables. These are updated by gps.c --------------

int32_t gpsLat, gpsLon;
uint8_t gpsHh, gpsMi, gpsSs, gpsDd, gpsMo, gpsYy;
uint8_t gpsUpdate;

// ------------------- Rest of main.c global variables --------------------

uint8_t appTimer;
uint8_t wdTimer;

uint8_t tempBuff[17];

uint8_t appState;
uint8_t gpsState;
uint8_t gpsN, gpsI;
uint32_t txInterval;
uint32_t goSleepInterval;


// ------------------------- Function prototypes -------------------------

void rxData(uint8_t* pData, uint8_t dataLength, OpStatus_t status);
void rxJoinResponse(bool status);
void exitFromSleep();
void appTimerCallback(uint8_t param);
void wdTimerCallback(uint8_t param);
void app_run();
uint16_t battery_voltage();
uint8_t battery_percent(uint16_t mvBatt);



void main(void)
{
    uint8_t data;
    
    // Initialize the device
	SYSTEM_Initialize();


#ifdef MCP9701
    // TXD (RC6): MCP9701 Vdd
    // RXD (RC7): MCP9701 Vout
    // Start with both pins pulled to GND
    TRISCbits.TRISC6 = 0;
    LATCbits.LATC6 = 0;
    TRISCbits.TRISC7 = 0;
    LATCbits.LATC7 = 0;
#endif
    
    // GPS pin initialize. This only happens once at device power-up
    // RC5 GPS Reset. Output, start at 1
    TRISCbits.RC5 = 0;
    LATCbits.LATC5 = 1;

    // RA6 GPS Enable. Output, start at 1
    TRISAbits.RA6 = 0;
    LATAbits.LATA6 = 1;

    gps_initialize();
    gps_sleep();
    
    // RD2 Battery measure enable. Output, start at 0
    TRISDbits.RD2 = 0;
    LATDbits.LATD2 = 0;
    // RE2 Battery half voltage. Digital output set to 0
    TRISEbits.RE2 = 0;
    ANSELEbits.ANSE2 = 0;
    LATEbits.LATE2 = 0;
    
    
    FVR_DeInitialize();
    
    SysSleepInit(exitFromSleep);
    
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    
    LORAWAN_Init(rxData, rxJoinResponse);
    LORAWAN_Reset(ISM_EU868);
    System_GetExternalEui(tempBuff);
    LORAWAN_SetDeviceEui(tempBuff);
    LORAWAN_SetApplicationEui(appEUI);
    LORAWAN_SetApplicationKey(appKey);

    
    appTimer = SwTimerCreate();
    SwTimerSetCallback(appTimer, appTimerCallback, 0);

    wdTimer = SwTimerCreate();
    SwTimerSetCallback(wdTimer, wdTimerCallback, 0);
    
    appState = STATE_APP_NOT_JOINED;
    txInterval = (uint32_t)2 * 60 * 1000;
    gpsState = STATE_GPS_OFF;
    gpsN = 0;
    gpsI = 0;


    gpsLat = -1000;
    gpsLon = -1000;
    gpsHh = 0;
    gpsMi = 0;
    gpsSs = 0;
    gpsDd = 0;
    gpsMo = 0;
    gpsYy = 0;
    gpsUpdate = 0;
    
    while (1)
    {
        LORAWAN_Mainloop();
       
        //Exit sleep mode (if possible)
        SysExitFromSleep();        

        app_run();
        
        //Enter sleep mode (if possible)
        SysGoToSleep();
        
        if (EUSART2_DataReady > 0)
        {
            data = EUSART2_Read();
            gps_parse(data);
            //EUSART1_Write(data);
        }
        
        if (EUSART1_DataReady > 0)
        {
            data = EUSART1_Read();
        }
        
    }
}


int8_t temperature()
{
    int32_t measurement;

    // TXD (RC6): MCP9701 Vdd
    // RXD (RC7): MCP9701 Vout

    // Make pin analog input
    TRISCbits.TRISC7 = 1;
    ANSELCbits.ANSC7 = 1;

    // Switch on Vdd
    LATCbits.LATC6 = 1;

    ADCMD = 0;
    ADC_Initialize();
    __delay_ms(2);
    measurement = ADC_GetConversion(19);
    ADCMD = 1;

    // Switch off Vdd
    LATCbits.LATC6 = 0;

    __delay_us(200);

    // Ground output pin
    TRISCbits.TRISC7 = 0;
    ANSELCbits.ANSC7 = 0;
    LATCbits.LATC7 = 0;

    
    measurement *= 3300;
    measurement -= 409600; // 1024 * 400 mV
    measurement /= 19968;  // 1024 * 19.5 mV/C
    
    return (int8_t)measurement;
}

uint16_t battery_voltage()
{
    uint32_t result;

    // Switch on FET
    LATDbits.LATD2 = 1;

    // RE2 Battery half voltage. Analog input
    TRISEbits.RE2 = 1;
    ANSELEbits.ANSE2 = 1;

    ADCMD = 0;
    ADC_Initialize();
    __delay_ms(2);
    ADC_GetConversion(7);
    __delay_ms(2);
    result = ADC_GetConversion(7);
    ADCMD = 1;

    // Switch off FET
    LATDbits.LATD2 = 0;

    // RE2 Battery half voltage. Digital output set to 0
    TRISEbits.RE2 = 0;
    ANSELEbits.ANSE2 = 0;
    LATEbits.LATE2 = 0;

    result *= 3300;
    result >>= 9;
    
    return result;
}

uint8_t battery_percent(uint16_t batt_mv)
{
	uint32_t val;

	if (batt_mv > 4054)
	{
		return 100;
	}
	else if (batt_mv < 3462)
	{
		return 0;
	}
	else
	{
		val = (uint32_t)batt_mv * 168;
		val -= 581500UL;
		val /= 1000UL;
		return (uint8_t) val;
	}
}

void rxJoinResponse(bool status)
{
    // Stop watchdog timer
    SwTimerStop(wdTimer);
    if (status)
    {
        EUSART1_Write('a');
        appState = STATE_APP_GO_SLEEP;
        goSleepInterval = 5000; // First packet in 5 seconds
    }
    else
    {
        EUSART1_Write('d');
        appState = STATE_APP_NOT_JOINED;
    }
}

void rxData(uint8_t* pData, uint8_t dataLength, OpStatus_t status)
{
    uint8_t i;
    EUSART1_Write('r');
    appState = STATE_APP_GO_SLEEP;
    goSleepInterval = txInterval;   // Application-configured interval until next Tx

    // Stop watchdog timer
    SwTimerStop(wdTimer);
    
    if (status != MAC_OK)
    {
        EUSART1_Write('0' + status);
        // Just complain
        //appState = STATE_APP_NOT_INIT;
    }
    
    if ((dataLength == 3) || (dataLength == 4))
    {
        // update interval in multiples of 5 minutes
        txInterval = (uint32_t)(pData[1] + 1) * 2 * 60 * 1000;

        // 0 - GPS off
        // 1 - GPS on for x packets
        // 2 - GPS on every x packets
        gpsState = pData[2];

        if ((gpsState == STATE_GPS_ON_FOR) && (dataLength ==4))
        {
            gpsN = 0;
            gpsI = pData[3];
        }
        else if ((gpsState == STATE_GPS_ON_EVERY) && (dataLength ==4))
        {
            gpsN = pData[3];
            gpsI = 0;
        }
    }
    
    if (dataLength == 6)
    {
        // Request rejoin in 15 minutes
        // C0FFEEBABE
        if ((pData[1] == 0xC0)
                && (pData[2] == 0xFF)
                && (pData[3] == 0xEE)
                && (pData[4] == 0xBA)
                && (pData[5] == 0xBE))
        {
            appState = STATE_APP_SLEEP_JOIN;
            SysSleepStart(900000);
        }
    }
}


void exitFromSleep()
{
    EUSART1_Write('w');
    switch (appState)
    {
        case STATE_APP_SLEEP_JOIN:
            appState = STATE_APP_NOT_JOINED;
            break;
        case STATE_APP_SLEEP:
            appState = STATE_APP_WAKE;
            break;
    }
    
}

void appTimerCallback(uint8_t param)
{
    EUSART1_Write('c');
    appState = STATE_APP_GPS_OFF;
}

void wdTimerCallback(uint8_t param)
{
    EUSART1_Write('q');
    // No clue what to do. Reset MAC, for lack of a better idea
    appState = STATE_APP_NOT_INIT;
}


LorawanError_t app_tx()
{
    static uint16_t counter = 0;
    static uint8_t battery = 0;
    static LorawanError_t status;
    
    tempBuff[0] = 0x80 + gpsState;

    tempBuff[1] = counter >> 8;
    tempBuff[2] = counter & 0xFF;

    battery = battery_percent(battery_voltage());
    tempBuff[3] = battery;
    
    tempBuff[4] = temperature();
    
    // Timestamp bit tetris
    // 7 bits: Year
    // 4 bits: Month
    // 5 bits: Day
    // 5 bits: Hour
    // 6 bits: Minute
    // 5 bits remaining
    tempBuff[5]  = (gpsYy & 0x7F) << 1;     // 7 bits of year
    tempBuff[5] |= (gpsMo & 0x08) >> 3;     // and 1 MSB of month
    tempBuff[6]  = (gpsMo & 0x07) << 5;     // 3 LSB of month
    tempBuff[6] |= (gpsDd & 0x1F);          // 5 bits of day
    tempBuff[7]  = (gpsHh & 0x1F) << 3;     // 5 bits of hour
    tempBuff[7] |= (gpsMi & 0x38) >> 3;     // 3 MSB of minute
    tempBuff[8]  = (gpsMi & 0x07) << 5;     // 3 LSB of minute

    tempBuff[9] = (gpsLat >> 24) & 0xFF;
    tempBuff[10] = (gpsLat >> 16) & 0xFF;
    tempBuff[11] = (gpsLat >> 8) & 0xFF;
    tempBuff[12] = gpsLat & 0xFF;

    tempBuff[13] = (gpsLon >> 24) & 0xFF;
    tempBuff[14] = (gpsLon >> 16) & 0xFF;
    tempBuff[15] = (gpsLon >> 8) & 0xFF;
    tempBuff[16] = gpsLon & 0xFF;

    if ((LORAWAN_GetState() == IDLE) && LORAWAN_CanSend())
    {
        LORAWAN_SetAdr(false);
        LORAWAN_SetCurrentDataRate(DR0);
        status = LORAWAN_Send(UNCNF, 10, tempBuff, 17);
        if (status == OK)
        {
            EUSART1_Write('t');
            counter++;
        }
    }
    else
    {
        status = NO_CHANNELS_FOUND;
    }
    
    return status;
}


void app_run()
{
    static LorawanError_t status;
    
    switch (appState)
    {
        case STATE_APP_NOT_INIT:
            LORAWAN_Init(rxData, rxJoinResponse);
            LORAWAN_Reset(ISM_EU868);
            System_GetExternalEui(tempBuff);
            LORAWAN_SetDeviceEui(tempBuff);
            LORAWAN_SetApplicationEui(appEUI);
            LORAWAN_SetApplicationKey(appKey);
            appState = STATE_APP_NOT_JOINED;
            break;
        case STATE_APP_NOT_JOINED:
            EUSART1_Write('J');
            if ((LORAWAN_GetState() == IDLE) && LORAWAN_CanSend())
            {
                if (LORAWAN_Join(OTAA) == OK)
                {
                    EUSART1_Write('j');
                    appState = STATE_APP_JOINING;
                    SwTimerSetTimeout(wdTimer, MS_TO_TICKS(15000));
                    SwTimerStart(wdTimer);
                }
            }
            else
            {
                appState = STATE_APP_SLEEP_JOIN;
                SysSleepStart(10000);
            }
            break;
        case STATE_APP_SLEEP_JOIN:
            // exit this state through exitFromSleep callback
            break;
        case STATE_APP_JOINING:
            // exit this state through rxJoinResponse callback
            break;
        case STATE_APP_GO_SLEEP:
            EUSART1_Write('S');
            appState = STATE_APP_SLEEP;
            SysSleepStart(goSleepInterval);
            break;
        case STATE_APP_SLEEP:
            // exit this state through exitFromSleep callback
            break;
        case STATE_APP_WAKE:
            EUSART1_Write('W');
            // Can we Tx? If not, no sense of trying to,
            // especially if we plan on turning GPS on
            if ((LORAWAN_GetState() == IDLE) && LORAWAN_CanSend())
            {
                EUSART1_Write('^');
                // do GPS or not?
                switch (gpsState)
                {
                    case STATE_GPS_OFF:
                        appState = STATE_APP_TX;
                        break;
                    case STATE_GPS_ON_FOR:
                        if (gpsI > 0)
                        {
                            appState = STATE_APP_GPS;
                            SwTimerSetTimeout(appTimer, MS_TO_TICKS(600000));
                            SwTimerStart(appTimer);
                            gpsUpdate = 0;
                            gps_initialize();
                            gpsI--;
                        }
                        else
                        {
                            appState = STATE_APP_TX;
                            gpsState = STATE_GPS_OFF;
                        }
                        break;
                    case STATE_GPS_ON_EVERY:
                        if (gpsI == 0)
                        {
                            appState = STATE_APP_GPS;
                            SwTimerSetTimeout(appTimer, MS_TO_TICKS(600000));
                            SwTimerStart(appTimer);
                            gpsUpdate = 0;
                            gps_initialize();
                            gpsI = gpsN;                        
                        }
                        else
                        {
                            gpsI--;
                            appState = STATE_APP_TX;
                        }
                        break;
                    default:
                        appState = STATE_APP_TX;
                        break;
                }
            }
            else
            {
                // We can't Tx right now, sleep some more. Try 10 seconds
                EUSART1_Write('v');
                appState = STATE_APP_GO_SLEEP;
                goSleepInterval = 5000;
            }
            break;
        case STATE_APP_GPS:
            // Can also exit this state through appTimerCallback callback
            if (gpsUpdate == 5)
            {
                appState = STATE_APP_GPS_OFF;
                SwTimerStop(appTimer);
            }
            break;
        case STATE_APP_GPS_OFF:
            EUSART1_Write('O');
            gps_sleep();
            appState = STATE_APP_TX;
            break;
        case STATE_APP_TX:
            EUSART1_Write('T');
            appState = STATE_APP_TX_WAIT;
            status = app_tx();
            SwTimerSetTimeout(wdTimer, MS_TO_TICKS(15000));
            SwTimerStart(wdTimer);
            switch (status)
            {
                case OK:
                    break;
                case NO_CHANNELS_FOUND:
                    appState = STATE_APP_GO_SLEEP;
                    goSleepInterval = txInterval;
                    SwTimerStop(wdTimer);
                    break;
                case NETWORK_NOT_JOINED:
                case FRAME_COUNTER_ERROR_REJOIN_NEEDED:
                    EUSART1_Write('Z');
                    appState = STATE_APP_NOT_JOINED;
                    SwTimerStop(wdTimer);
                    break;
                case MAC_STATE_NOT_READY_FOR_TRANSMISSION:
                    EUSART1_Write('Y');
                    break;
                case INVALID_PARAMETER:
                case KEYS_NOT_INITIALIZED:
                case SILENT_IMMEDIATELY_ACTIVE:
                case INVALID_BUFFER_LENGTH:
                case MAC_PAUSED:
                case INVALID_CLASS:
                    EUSART1_Write('Q');
                    break;
                case MCAST_PARAM_ERROR:
                case MCAST_MSG_ERROR:
                    EUSART1_Write('X');
                    break;
            }
            break;
        case STATE_APP_TX_WAIT:
            // exit this state through rxData callback
            break;
    }
}

/**
 End of File
 */
