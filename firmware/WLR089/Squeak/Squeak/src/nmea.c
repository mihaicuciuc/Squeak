#include <stdint.h>
#include "nmea.h"

extern int32_t gpsLat, gpsLon;
extern uint16_t gpsHdop;
extern uint8_t gpsHh, gpsMi, gpsSs, gpsDd, gpsMo, gpsYy;
extern uint8_t gpsUpdate;

static uint8_t hex_nibble(uint8_t a);
static uint32_t gps_parse_int(uint8_t* buf, uint8_t pad);
static uint8_t gps_parse_int_2dig(uint8_t* buf);
static void gps_parse_gprmc(uint8_t data);
static void gps_parse_gpgga(uint8_t data);


uint8_t hex_nibble(uint8_t a)
{
    if (a < 10) return a + '0';
    else return a - 10 + 'A';
}

uint32_t gps_parse_int(uint8_t* buf, uint8_t max)
{
	uint32_t retVal;
	uint8_t i;
	retVal = 0;
	i = 0;
	while (buf[i] != 0)
	{
		if ((buf[i] != '.') && (max != 0))
		{
			retVal *= 10;
			retVal += buf[i] - '0';
			max--;
		}
		
		i++;
	}
	for (i = 0; i < max; i++)
	{
		retVal *= 10;
	}
	return retVal;
}

uint32_t gps_parse_fixed_point(uint8_t* buf, uint8_t decimals)
{
	uint32_t retVal;
	uint8_t i;
	retVal = 0;
	i = 0;
	while ((buf[i] != 0) && (buf[i] != '.'))
	{
		retVal *= 10;
		retVal += buf[i] - '0';
		i++;
	}

	if (buf[i] == '.') i++;

	buf += i;

	for (i = 0; i < decimals; i++)
	{
		retVal *= 10;
		if (*buf != 0)
		{
			retVal += *buf - '0';
			buf++;
		}
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
	static uint8_t bufRMC[] = "GNRMC,";
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
			else
			{
				state = STATE_RMC_IDLE;
			}
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
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
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
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
			}
			break;
		case STATE_RMC_LAT:
			if (data == ',')
			{
				buf[bufPtr] = 0;
				if (bufPtr != 0) intLat = gps_parse_int(buf, 9);
				bufPtr = 0;
				state = STATE_RMC_LAT_NS;
			}
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
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
			else if (data == 'N')
			{
				// nothing
			}
			else
			{
				state = STATE_RMC_IDLE;
			}
			break;
		case STATE_RMC_LON:
			if (data == ',')
			{
				buf[bufPtr] = 0;
				if (bufPtr != 0) intLon = gps_parse_int(buf, 10);
				bufPtr = 0;
				state = STATE_RMC_LON_EW;
			}
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
			}
			break;
		case STATE_RMC_LON_EW:
			if (data == ',')
			{
				bufPtr = 0;
				state = STATE_RMC_SPEED;
			}
			else if (data == 'W')
			{
				intLon = -intLon;
			}
			else if (data == 'E')
			{
				// nothing
			}
			else
			{
				state = STATE_RMC_IDLE;
			}
			break;
		case STATE_RMC_SPEED:
			if (data == ',')
			{
				bufPtr = 0;
				state = STATE_RMC_TRACK;
			}
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
			}
			break;
		case STATE_RMC_TRACK:
			if (data == ',')
			{
				bufPtr = 0;
				state = STATE_RMC_DATE;
			}
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
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
			else if (bufPtr < 20)
			{
				buf[bufPtr] = data;
				bufPtr++;
			}
			else
			{
				state = STATE_RMC_IDLE;
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
			// What if * never comes?
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
					//EUSART1_Write('g');
				}
			}
			state = STATE_RMC_IDLE;
			break;
		default:
			state = STATE_RMC_IDLE;
			break;
	}
}



#define STATE_GGA_IDLE      0
#define STATE_GGA_START     1
#define STATE_GGA_TS        2
#define STATE_GGA_LAT       3
#define STATE_GGA_LAT_NS    4
#define STATE_GGA_LON       5
#define STATE_GGA_LON_EW    6
#define STATE_GGA_QUALITY   7
#define STATE_GGA_NSATS     8
#define STATE_GGA_HDOP      9
#define STATE_GGA_REST      10
#define STATE_GGA_CHECK_0   11
#define STATE_GGA_CHECK_1   12

void gps_parse_gpgga(uint8_t data)
{
    static uint8_t state = STATE_GGA_IDLE;
    static uint8_t bufPtr = 0;
    static uint8_t bufGGA[] = "GNGGA,";
    static uint8_t buf[20];
    static int32_t intLat, intLon;
    static uint16_t intHdop;
    static uint8_t checksum = 0;

    checksum ^= data;

    switch (state)
    {
        case STATE_GGA_IDLE:
            if (data == '$')
            {
                state = STATE_GGA_START;
                bufPtr = 0;
                intHdop = 65000;
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
            else
			{
				state = STATE_GGA_IDLE;
			}
            break;
        case STATE_GGA_TS:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_LAT;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_LAT:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_LAT_NS;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_LAT_NS:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_LON;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_LON:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_LON_EW;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_LON_EW:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_QUALITY;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_QUALITY:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_NSATS;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_NSATS:
            if (data == ',')
            {
				bufPtr = 0;
                state = STATE_GGA_HDOP;
            }
            else if (bufPtr < 20)
            {
	            buf[bufPtr] = data;
	            bufPtr++;
            }
            else
            {
	            state = STATE_GGA_IDLE;
            }
            break;
        case STATE_GGA_HDOP:
            if (data == ',')
            {
                buf[bufPtr] = 0;
                if (bufPtr != 0) intHdop = gps_parse_fixed_point(buf, 2);
                bufPtr = 0;
                state = STATE_GGA_REST;
            }
            else if (bufPtr < 20)
            {
                buf[bufPtr] = data;
                bufPtr++;
            }
			else
			{
				state = STATE_GGA_IDLE;
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
			// What if * never comes?
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
                if (intHdop != 65000)
                {
	                gpsHdop = intHdop;
                }
            }
            state = STATE_GGA_IDLE;
            break;
        default:
            state = STATE_GGA_IDLE;
            break;
    }
}




void gps_parse(uint8_t data)
{
    gps_parse_gprmc(data);
    gps_parse_gpgga(data);
}
