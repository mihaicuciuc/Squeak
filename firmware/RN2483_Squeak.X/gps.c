#include "mcc_generated_files/mcc.h"

extern int32_t gpsLat, gpsLon;
extern uint8_t gpsHh, gpsMi, gpsSs, gpsDd, gpsMo, gpsYy;
extern uint8_t gpsUpdate;


uint8_t hex_nibble(uint8_t a)
{
    if (a < 10) return a + '0';
    else return a - 10 + 'A';
}

uint32_t gps_parse_int(uint8_t* buf, uint8_t pad)
{
    uint32_t retVal;
    uint8_t i, frac;
    retVal = 0;
    i = 0;
    frac = 0;
    while (buf[i] != 0)
    {
        if (frac == 1)
        {
            pad--;
        }
        
        if (buf[i] != '.')
        {
            retVal *= 10;
            retVal += buf[i] - '0';
        }
        else
        {
            frac = 1;
        }
        
        i++;
    }
    for (i = 0; i < pad; i++)
    {
        retVal *= 10;
    }
    return retVal;
}

uint8_t gps_parse_int_2dig(uint8_t* buf)
{
    return (buf[0] - '0') * 10 + (buf[1] - '0');
}


#define STATE_RMC_IDLE      0
#define STATE_RMC_START     1
#define STATE_RMC_TS        2
#define STATE_RMC_STATUS    3
#define STATE_RMC_LAT       4
#define STATE_RMC_LAT_NS    5
#define STATE_RMC_LON       6
#define STATE_RMC_LON_EW    7
#define STATE_RMC_SPEED     8
#define STATE_RMC_TRACK     9
#define STATE_RMC_DATE      10
#define STATE_RMC_REST      11
#define STATE_RMC_CHECK_0   12
#define STATE_RMC_CHECK_1   13


void gps_parse_gprmc(uint8_t data)
{
    static uint8_t state = STATE_RMC_IDLE;
    static uint8_t bufPtr = 0;
    static uint8_t bufRMC[] = "GPRMC,";
    static uint8_t buf[20];
    static int32_t intLat, intLon;
    static uint8_t intStatus, intHh, intMi, intSs, intDd, intMo, intYy;
    static uint8_t checksum = 0;

    checksum ^= data;

    switch (state)
    {
        case STATE_RMC_IDLE:
            if (data == '$')
            {
                state = STATE_RMC_START;
                bufPtr = 0;
                intLat = -1000;
                intLon = -1000;
                intStatus = 0;
                intHh = 255;
                intMi = 255;
                intSs = 255;
                intDd = 255;
                intMo = 255;
                intYy = 255;
                checksum = 0;
            }
            break;
        case STATE_RMC_START:
            if (data == bufRMC[bufPtr])
            {
                bufPtr++;
                if (bufPtr == sizeof(bufRMC) - 1)
                {
                    bufPtr = 0;
                    state = STATE_RMC_TS;
                }
            }
            else state = STATE_RMC_IDLE;
            break;
        case STATE_RMC_TS:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr >= 6)
                {
                    intHh = gps_parse_int_2dig(buf);
                    intMi = gps_parse_int_2dig(&buf[2]);
                    intSs = gps_parse_int_2dig(&buf[4]);
                }
                bufPtr = 0;
                state = STATE_RMC_STATUS;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_RMC_STATUS:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0)
                {
                    if (buf[0] == 'A') intStatus = 1;
                }
                bufPtr = 0;
                state = STATE_RMC_LAT;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;            
        case STATE_RMC_LAT:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intLat = gps_parse_int(buf, 5);
                bufPtr = 0;
                state = STATE_RMC_LAT_NS;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_RMC_LAT_NS:
            if (data == ',')
            {
                bufPtr = 0;
                state = STATE_RMC_LON;
            }
            else if (data == 'S')
            {
                intLat = -intLat;
            }
            break;
        case STATE_RMC_LON:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intLon = gps_parse_int(buf, 5);
                bufPtr = 0;
                state = STATE_RMC_LON_EW;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_RMC_LON_EW:
            if (data == ',')
            {
                state = STATE_RMC_SPEED;
            }
            else if (data == 'W')
            {
                intLon = -intLon;
            }
            break;
        case STATE_RMC_SPEED:
            if (data == ',')
            {
                state = STATE_RMC_TRACK;
            }
            break;
        case STATE_RMC_TRACK:
            if (data == ',')
            {
                state = STATE_RMC_DATE;
            }
            break;
        case STATE_RMC_DATE:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr >= 6)
                {
                    intDd = gps_parse_int_2dig(buf);
                    intMo = gps_parse_int_2dig(&buf[2]);
                    intYy = gps_parse_int_2dig(&buf[4]);
                }
                bufPtr = 0;
                state = STATE_RMC_REST;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_RMC_REST:
            if (data == '*')
            {
                // end of sentence. Check checksum
                checksum ^= data; // we need to undo default xor
                bufPtr = 0;
                state = STATE_RMC_CHECK_0;
            }
            break;
        case STATE_RMC_CHECK_0:
            checksum ^= data; // we need to undo default xor
            if (data == hex_nibble(checksum >> 4)) state = STATE_RMC_CHECK_1;
            else state = STATE_RMC_IDLE;
            break;
        case STATE_RMC_CHECK_1:
            checksum ^= data; // we need to undo default xor
            if (data == hex_nibble(checksum & 0x0F))
            {
                // Checksum ok. Update values if they were available
                if ((intLat != -1000) && (intLon != -1000) && (intStatus == 1))
                {
                    gpsLat = intLat;
                    gpsLon = intLon;
                    gpsHh = intHh;
                    gpsMi = intMi;
                    gpsSs = intSs;
                    gpsDd = intDd;
                    gpsMo = intMo;
                    gpsYy = intYy;
                    gpsUpdate++;
                    EUSART1_Write('g');
                }
            }
            state = STATE_RMC_IDLE;
            break;
        default:
            break;
    }
}


inline void gps_parse(uint8_t data)
{
    gps_parse_gprmc(data);
}

void gps_sleep()
{
    // Switch off voltage regulator feeding GPS
    LATAbits.LATA6 = 0;

    // Stop EUSART2
    TXEN2 = 0;
    CREN2 = 0;
    SPEN2 = 0;
    UART2MD = 1;
    
    // Configure IO pins accordingly - RD7 - dig. in, RD6 - dig. out set low
    TRISDbits.TRISD7 = 1;
    TRISDbits.TRISD6 = 0;
    LATDbits.LATD6 = 0;

    __delay_ms(1);  // wait for capacitor to discharge, don't want to short pin to GND just yet
    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;

    // Pull reset pin low
    LATCbits.LATC5 = 0;
}

void gps_initialize()
{
    UART2MD = 0;
    // initialize RX and TX pins done by enabling EUSART2
    TRISDbits.TRISD7 = 1;
    TRISDbits.TRISD6 = 0;
    LATDbits.LATD6 = 1;
    EUSART2_Initialize();

    // Switch on voltage regulator
    LATAbits.LATA6 = 1;
    
    // Bring reset pin high
    LATCbits.LATC5 = 1;
}






/*
#define STATE_GGA_IDLE      0
#define STATE_GGA_START     1
#define STATE_GGA_TS        2
#define STATE_GGA_LAT       3
#define STATE_GGA_LAT_NS    4
#define STATE_GGA_LON       5
#define STATE_GGA_LON_EW    6
#define STATE_GGA_REST      7
#define STATE_GGA_CHECK_0   8
#define STATE_GGA_CHECK_1   9

void gps_parse_gpgga(uint8_t data)
{
    static uint8_t state = STATE_GGA_IDLE;
    static uint8_t bufPtr = 0;
    static uint8_t bufGGA[] = "GPGGA,";
    static uint8_t buf[20];
    static int32_t intTs, intLat, intLon;
    static uint8_t checksum = 0;

    checksum ^= data;

    switch (state)
    {
        case STATE_GGA_IDLE:
            if (data == '$')
            {
                state = STATE_GGA_START;
                bufPtr = 0;
                intTs = -1000;
                intLat = -1000;
                intLon = -1000;
                checksum = 0;
            }
            break;
        case STATE_GGA_START:
            if (data == bufGGA[bufPtr])
            {
                bufPtr++;
                if (bufPtr == sizeof(bufGGA) - 1)
                {
                    bufPtr = 0;
                    state = STATE_GGA_TS;
                }
            }
            else state = STATE_GGA_IDLE;
            break;
        case STATE_GGA_TS:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intTs = gps_parse_int(buf, 3);
                bufPtr = 0;
                state = STATE_GGA_LAT;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_GGA_LAT:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intLat = gps_parse_int(buf, 5);
                bufPtr = 0;
                state = STATE_GGA_LAT_NS;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_GGA_LAT_NS:
            if (data == ',')
            {
                bufPtr = 0;
                state = STATE_GGA_LON;
            }
            else if (data == 'S')
            {
                intLat = -intLat;
            }
            break;
        case STATE_GGA_LON:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intLon = gps_parse_int(buf, 5);
                bufPtr = 0;
                state = STATE_GGA_LON_EW;
            }
            else
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
            break;
        case STATE_GGA_LON_EW:
            if (data == ',')
            {
                state = STATE_GGA_REST;
            }
            else if (data == 'W')
            {
                intLon = -intLon;
            }
            break;
        case STATE_GGA_REST:
            if (data == '*')
            {
                // end of sentence. Check checksum
                checksum ^= data; // we need to undo default xor
                bufPtr = 0;
                state = STATE_GGA_CHECK_0;
            }
            break;
        case STATE_GGA_CHECK_0:
            checksum ^= data; // we need to undo default xor
            if (data == hex_nibble(checksum >> 4)) state = STATE_GGA_CHECK_1;
            else state = STATE_GGA_IDLE;
            break;
        case STATE_GGA_CHECK_1:
            checksum ^= data; // we need to undo default xor
            if (data == hex_nibble(checksum & 0x0F))
            {
                // Checksum ok. Update values if they were available
                if ((intLat != -1000) && (intLon != -1000))
                {
                    gpsTs = intTs;
                    gpsLat = intLat;
                    gpsLon = intLon;
                    gpsUpdate++;
                    EUSART1_Write('g');
                }
            }
            state = STATE_GGA_IDLE;
            break;
        default:
            break;
    }
}
*/

