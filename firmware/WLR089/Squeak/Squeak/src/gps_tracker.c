/**
* \file  enddevice_demo.c
*
* \brief Getting Started LoRaWAN [USB_CDC] Demo Application
*
*
* Copyright (c) 2019-2020 Microchip Technology Inc. and its subsidiaries.
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products.
* It is your responsibility to comply with third party license terms applicable
* to your use of third party software (including open source software) that
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

//#define MMC
//#define GPSTTFF


/****************************** INCLUDES **************************************/
#include "asf.h"
#include "lorawan.h"
#include "system_task_manager.h"
#include "gps_tracker.h"
#include "conf_app.h"
#include "sio2host.h"
#include "delay.h"
#include "sw_timer.h"
#include "radio_driver_hal.h"
#include "radio_interface.h"
#include "conf_sio2host.h"
#include "pds_interface.h"
#include "lorawan_reg_params.h"
#include "conf_regparams.h"
#include "lorawan_multiband.h"
#include "lorawan_private.h"

#include "conf_pmm.h"
#include "pmm.h"
#include "sleep.h"

#include "atomic.h"
#include <stdint.h>

#include "nmea.h"

#ifdef MMC
#include "fatFS/ff.h"		/* Declarations of FatFs API */
FATFS FatFs;		/* FatFs work area needed for each volume */
FIL Fil;			/* File object needed for each open file */

#define ACC_BUFFERS		9
static int16_t acc_data_x[ACC_BUFFERS*150], acc_data_y[ACC_BUFFERS*150], acc_data_z[ACC_BUFFERS*150];
volatile static uint8_t acc_buffer;
volatile static bool mmc_flush_file;
static uint32_t acc_file;
static bool mmc_mounted;

bool mmc_mount(void);
bool mmc_unmount(void);
void mmc_write_file(void);

#endif

/************************** GLOBAL VARIABLES ***********************************/
extern LoRa_t loRa;

extern volatile bool usb_connected;
extern volatile bool isr_button;
extern volatile bool isr_vbus;
extern volatile bool isr_accel;

static volatile bool usb_power;

static uint8_t digInputTimerId = 0xFF;

static uint8_t ledTimerId = 0xFF;

static uint8_t appTimerId = 0xFF;
static uint8_t wdTimerId = 0xFF;
static uint8_t p2pDcycleTimerId = 0xFF;

static AppState_t appState;
static GpsState_t gpsState;
static UsbState_t usbState;
static P2PState_t p2pState;

static uint8_t gpsN, gpsI;
static uint32_t txInterval;
static uint32_t txIntervalLocate;
static int32_t sleepInterval;
static uint16_t txCounter;
static uint8_t nmeaEcho;

static uint64_t timeOnAir;
static uint64_t p2pStopTime;

static char pstr1[80];

/* OTAA join parameters */
static uint8_t devEui[8] = DEVICE_EUI;
static uint8_t joinEui[8] = APPLICATION_EUI;
static uint8_t appKey[16] = APPLICATION_KEY;

static LorawanSendReq_t lorawanSendReq;
static uint8_t txBuffer[32];


int32_t gpsLat, gpsLon;
uint16_t gpsHdop;
uint8_t gpsHh, gpsMi, gpsSs, gpsDd, gpsMo, gpsYy;
uint8_t gpsUpdate;

uint32_t stepsCounter;
uint64_t lastTxTs;

static uint8_t tempSensorType;


#ifdef GPSTTFF
typedef struct
{
	int32_t lat;
	int32_t lon;
	union
	{
		uint32_t ts;
		uint8_t tsBytes[4];
	};
} gpsRecord_t;

gpsRecord_t gpsRecord[1200];
uint16_t gpsRecords;
uint8_t gpsNoSleep;
#endif


// Ugh... :((
int32_t Radio_ReadFei(void);

/*********************************** I2C ***************************************/
#define I2C_ADDR_PCT2075	0x48
#define I2C_ADDR_TMP112		0x49

static struct i2c_master_module i2c_instance_app;
static struct adc_module adc_instance_app;
struct spi_module spi_instance_app;

static void i2c_init_temp(void);
static int8_t i2c_read_temp(void);


static void spi_init_accel(void);
static void spi_write_reg_accel(uint8_t addr, uint8_t data);
static uint8_t spi_read_reg_accel(uint8_t addr);
static void spi_read_fifo_accel(uint16_t size, int16_t* buf_x, int16_t* buf_y, int16_t* buf_z);
static void spi_read_samples_accel(void);

static void adc_init_vbat(void);
static uint16_t adc_read_vbat(void);
static uint8_t battery_percent(uint16_t batt_mv);

/************************** FUNCTION PROTOTYPES ********************************/
// Dummy function as this mechanism is unused in this project
SYSTEM_TaskStatus_t APP_TaskHandler(void);
SYSTEM_TaskStatus_t APP_TaskHandler(void) { return SYSTEM_TASK_SUCCESS; }

void gps_sleep(void);
void gps_wake(void);

#ifdef GPSTTFF
void gps_cold_boot(void);
#endif

static void ledStop(void);
static void ledBlink(void);

P2PStatus_t app_lorawan(void);
P2PStatus_t app_p2p(void);
P2PStatus_t app_p2p_tx_downlink(uint8_t dataLen);
P2PStatus_t app_p2p_rx_downlink(void);
P2PStatus_t app_p2p_tx_uplink(uint8_t dataLen);
P2PStatus_t app_p2p_rx_uplink(void);


static StackRetStatus_t app_init_lora(void);
static uint32_t app_sleep(uint32_t sleepTime);
static volatile uint32_t app_slept_duration;
static void app_wake(uint32_t sleptDuration);
static void reset_to_bootloader(void);

void ledTimerCb(void *data);
static void digInputTimerCb(void *data);
static void appTimerCb(void *data);
StackRetStatus_t app_tx();

void app_tx_callback(void *appHandle, appCbParams_t *appdata);
static void app_rx_data(void *appHandle, appCbParams_t *appdata);
void app_join_callback(StackRetStatus_t status);
void gpsTimerCallback(void);
void wdTimerCallback(void);

static void read_deveui(void);

extern void USB_printf(const char* str);
extern int USB_sprintf(char * str, const char * format, ... );

/***************************** FUNCTIONS ***************************************/


static void i2c_init_temp(void)
{
	enum status_code status;
	struct i2c_master_config config_i2c_master;
	uint8_t i2c_buffer[3];
	struct i2c_master_packet master_packet = {
		.address     = I2C_ADDR_TMP112,
		.data_length = 3,
		.data        = &i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = 100;
	config_i2c_master.pinmux_pad0    = PINMUX_PA16D_SERCOM3_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA17D_SERCOM3_PAD1;
	config_i2c_master.run_in_standby = 0;
	status = i2c_master_init(&i2c_instance_app, SERCOM3, &config_i2c_master);
	//USB_sprintf(pstr1, "I2C master init returned %02X\r\n", status);
	i2c_master_enable(&i2c_instance_app);

	USB_printf("Probing I2C bus for TMP112.. ");

	master_packet.data_length = 1;
	i2c_buffer[0] = 0; // temperature register
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);

	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);

	if (STATUS_OK == status)
	{
		USB_printf("found TMP112\r\n");
		
		master_packet.data_length = 3;
		i2c_buffer[0] = 1; // write to conf register @ addr. 1. Active, 0.25 Hz
		i2c_buffer[1] = 0x60;
		i2c_buffer[2] = 0x20;
		status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);

		master_packet.data_length = 1;
		i2c_buffer[0] = 0; // Switch pointer to temperature register
		status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);

		tempSensorType = 0;
		return;
	}

	USB_printf("\r\nProbing I2C bus for PCT2075.. ");

	master_packet.address = I2C_ADDR_PCT2075;
	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);

	if (STATUS_OK == status)
	{
		USB_printf("found PCT2075\r\n");
		tempSensorType = 1;
		return;
	}

	tempSensorType = 255;
	return;
}

static int8_t i2c_read_temp_PCT2075(void)
{
	enum status_code status;
	uint8_t i2c_buffer[2];
	int8_t temperature;
	struct i2c_master_packet master_packet = {
		.address     = I2C_ADDR_PCT2075,
		.data_length = 2,
		.data        = i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	i2c_buffer[0] = 1; // write to conf register @ addr. 1
	i2c_buffer[1] = 0; // value 0x00 to turn sensor on

	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
	//USB_sprintf(pstr1, "i2c_master_write_packet_wait returned %02X\r\n", status);

	delay_ms(100); // wait for measurement

	i2c_buffer[0] = 0; // read from temperaure register @ addr 0
	master_packet.data_length = 1;

	status = i2c_master_write_packet_wait_no_stop(&i2c_instance_app, &master_packet);
	//USB_sprintf(pstr1, "i2c_master_write_packet_wait_no_stop returned %02X\r\n", status);

	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
	temperature = (int8_t)i2c_buffer[0];

	//USB_sprintf(pstr1, "i2c_master_read_packet_wait returned %02X\r\n", status);
	//USB_sprintf(pstr1, "Temperature: %d\r\n", temperature);

	// Turn sensor off
	master_packet.data_length = 2;
	i2c_buffer[0] = 1; // write to conf register @ addr. 1
	i2c_buffer[1] = 1; // value 0x01 enter shutdown state

	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
	//USB_sprintf(pstr1, "i2c_master_write_packet_wait returned %02X\r\n", status);

	return temperature;
}


static int8_t i2c_read_temp_TMP112(void)
{
	enum status_code status;
	uint8_t i2c_buffer[3];
	int8_t temperature;
	struct i2c_master_packet master_packet = {
		.address     = I2C_ADDR_TMP112,
		.data_length = 3,
		.data        = i2c_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

/*
	i2c_buffer[0] = 1;
	i2c_buffer[1] = 0xE1;
	i2c_buffer[2] = 0xE0;
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);


	master_packet.data_length = 2;
	int8_t i;
	for (i = 0; i < 100; i++)
	{
		status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
		USB_sprintf(pstr1, "CFG: %02X %02X\r\n", i2c_buffer[0], i2c_buffer[1]);
		delay_ms(1);
	}
	//USB_sprintf(pstr1, "i2c_master_write_packet_wait returned %02X\r\n", status);

	delay_ms(200); // conversion rate set to 8 Hz. So in 125ms we should have gotten a new reading

	i2c_buffer[0] = 0; // read from temperaure register @ addr 0
	master_packet.data_length = 1;

	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
	//USB_sprintf(pstr1, "i2c_master_write_packet_wait returned %02X\r\n", status);
*/
	status = i2c_master_read_packet_wait(&i2c_instance_app, &master_packet);
	temperature = (int8_t)i2c_buffer[0];

/*
	master_packet.data_length = 3;
	i2c_buffer[0] = 1;
	i2c_buffer[1] = 0x61;	// set shutdown bit
	i2c_buffer[2] = 0xE0;
	status = i2c_master_write_packet_wait(&i2c_instance_app, &master_packet);
*/

	return temperature;
}

static int8_t i2c_read_temp(void)
{
	if (tempSensorType == 0) return i2c_read_temp_TMP112();
	else if (tempSensorType == 1) return i2c_read_temp_PCT2075();
	else return -128;
}



static void spi_init_accel(void)
{
	struct spi_config config;
	
	spi_get_config_defaults(&config);
	config.mux_setting     = SPI_SIGNAL_MUX_SETTING_E;
	config.pinmux_pad0     = PINMUX_PB02D_SERCOM5_PAD0;
	config.pinmux_pad1     = PINMUX_UNUSED;
	config.pinmux_pad2     = PINMUX_PB22D_SERCOM5_PAD2;
	config.pinmux_pad3     = PINMUX_PB23D_SERCOM5_PAD3;
	config.run_in_standby  = 0;
	config.transfer_mode   = SPI_TRANSFER_MODE_0;
	config.mode_specific.master.baudrate = 4000000UL;
	spi_init(&spi_instance_app, SERCOM5, &config);
	spi_enable(&spi_instance_app);
	
	spi_write_reg_accel(0x28, 0x02 | 0x08);		// FIFO control. FIFO in stream mode. Plus MSB of FIFO samples = 1
	spi_write_reg_accel(0x29, 0xC2);			// FIFO samples. Set to 450 (6 seconds x 3 values x 25 Hz)
	//spi_write_reg_accel(0x2A, 0x04);			// INTMAP1. Int active high, set to FIFO_WATERMARK
	spi_write_reg_accel(0x2A, 0x84);			// INTMAP1. Int active low, set to FIFO_WATERMARK
	spi_write_reg_accel(0x2C, 0x11);			// Filter control. HALF_BW = 1, output data rate = 25Hz
	spi_write_reg_accel(0x2D, 0x02);			// Power control. Measurement mode
}

static void spi_write_reg_accel(uint8_t addr, uint8_t data)
{
	uint16_t rx_data;
	// Assert /CS
	port_pin_set_output_level(ACCCS_PIN, 0);
	spi_transceive_wait(&spi_instance_app, 0x0A, &rx_data);
	spi_transceive_wait(&spi_instance_app, addr, &rx_data);
	spi_transceive_wait(&spi_instance_app, data, &rx_data);
	port_pin_set_output_level(ACCCS_PIN, 1);
}

static uint8_t spi_read_reg_accel(uint8_t addr)
{
	static uint16_t rx_data;
	// Assert /CS
	port_pin_set_output_level(ACCCS_PIN, 0);
	spi_transceive_wait(&spi_instance_app, 0x0B, &rx_data);
	spi_transceive_wait(&spi_instance_app, addr, &rx_data);
	spi_transceive_wait(&spi_instance_app, 0x00, &rx_data);
	port_pin_set_output_level(ACCCS_PIN, 1);
	return (uint8_t)rx_data;
}

static void spi_read_fifo_accel(uint16_t size, int16_t* buf_x, int16_t* buf_y, int16_t* buf_z)
{
	static uint16_t i, rx_data, currentSample;
	static int16_t intSample;
	static uint8_t j;
	// Assert /CS
	port_pin_set_output_level(ACCCS_PIN, 0);
	spi_transceive_wait(&spi_instance_app, 0x0D, &rx_data);
	for (i = 0; i < size; i++)
	{
		for (j = 0; j < 3; j++)
		{
			spi_transceive_wait(&spi_instance_app, 0x00, &rx_data);
			currentSample = (uint8_t)rx_data;
			spi_transceive_wait(&spi_instance_app, 0x00, &rx_data);
			currentSample |= rx_data << 8;
			rx_data >>= 6;
			currentSample <<= 2;
			intSample = (int16_t)currentSample;
			intSample >>= 2;
			switch(rx_data)
			{
				case 0:
					buf_x[i] = intSample;
					break;
				case 1:
					buf_y[i] = intSample;
					break;
				case 2:
					buf_z[i] = intSample;
					break;
			}
		}
	}
	port_pin_set_output_level(ACCCS_PIN, 1);
}

/**
https://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
 */
uint32_t SquareRoot(uint32_t a_nInput)
{
    uint32_t op  = a_nInput;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


    // "one" starts at the highest power of four <= than the argument.
    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

#define STEP_TRHRESHOLD		-150
#define STEP_HOLDOFF		3
static uint16_t step_count(uint16_t data_n, int16_t *data_x, int16_t *data_y, int16_t *data_z)
{
	uint16_t i, steps;
	uint32_t avg;
	uint32_t temp;
	uint8_t holdoff, armed;
	
	avg = 0;
	for (i = 0; i < data_n; i++)
	{
		temp = (uint32_t)data_x[i]*data_x[i] + (uint32_t)data_y[i]*data_y[i] + (uint32_t)data_z[i]*data_z[i];
		data_x[i] = SquareRoot(temp);
		avg += data_x[i];
	}
	
	avg /= data_n;

	holdoff = 0;
	armed = 0;
	steps = 0;
	data_x[0] -= avg;
	for (i = 1; i < data_n; i++)
	{
		data_x[i] -= avg;
		
		if (holdoff > 0) holdoff--;
		if (data_x[i] < STEP_TRHRESHOLD) armed = 1;
		if ((data_x[i-1] <= 0) && (data_x[i] > 0) && (holdoff == 0) && (armed == 1))
		{
			steps++;
			holdoff = STEP_HOLDOFF + 1;
			armed = 0;
		}
	}
	
	return steps;
}

static void spi_read_samples_accel(void)
{
	uint16_t samples, i, buffer_idx;
	int16_t data_x[150], data_y[150], data_z[150];
	
	samples = spi_read_reg_accel(0x0D) & 0x03;
	samples <<= 8;
	samples |= spi_read_reg_accel(0x0C);
	
	if (samples > 450) samples = 450;
	samples /= 3;
	
	spi_read_fifo_accel(samples, data_x, data_y, data_z);

#ifdef MMC
	if (samples < 150)
	{
		// We have to pad with something, as we got insufficient data from accelerometer
		for (i = samples; i < 150; i++)
		{
			data_x[i] = 32767;
			data_y[i] = 32767;
			data_z[i] = 32767;
		}
	}

	memcpy(&acc_data_x[acc_buffer*150], &data_x, 300);
	memcpy(&acc_data_y[acc_buffer*150], &data_y, 300);
	memcpy(&acc_data_z[acc_buffer*150], &data_z, 300);

	//USB_sprintf(pstr1, "\r\nacc_buffer == %d\r\n", acc_buffer);
	
	if (acc_buffer < (ACC_BUFFERS - 1)) acc_buffer++;

	// LoRaWAN stack may be busy so we're going to preemptively flush buffers
	// before they're FULL full
	if (acc_buffer >= (ACC_BUFFERS - 2))
	{
		// Full, dump to card at your convenience
		mmc_flush_file = true;
	}

#endif

	// compute step count. This function messes up the data_x array
	stepsCounter += step_count(samples, data_x, data_y, data_z);
	//LED_Off(LED0_PIN);

/*	
	USB_printf("X:");
	for (i = 0; i < samples / 3; i++) USB_sprintf(pstr1, " %d", data_x[i]);
	USB_printf("\r\nY:");
	for (i = 0; i < samples / 3; i++) USB_sprintf(pstr1, " %d", data_y[i]);
	USB_printf("\r\nZ:");
	for (i = 0; i < samples / 3; i++) USB_sprintf(pstr1, " %d", data_z[i]);
	USB_printf("\r\n");
*/
}

static void adc_init_vbat(void)
{
	struct adc_config conf_adc;
				
	adc_get_config_defaults(&conf_adc);
				
	conf_adc.clock_source = GCLK_GENERATOR_2;
	conf_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV2;
	conf_adc.reference = ADC_REFCTRL_REFSEL_INTREF;
	conf_adc.positive_input = ADC_POSITIVE_INPUT_PIN11;
	conf_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	conf_adc.sample_length = 63;
	conf_adc.run_in_standby = 0;
	conf_adc.resolution = ADC_RESOLUTION_12BIT;
				
	adc_init(&adc_instance_app, ADC, &conf_adc);
				
	system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_OUTPUT);
	system_voltage_reference_disable(SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE);
				
	adc_enable(&adc_instance_app);
}

static uint16_t adc_read_vbat(void)
{
	uint32_t batt_mv;
	uint16_t adc_result = 0;
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(HBAT_PIN, &pin_conf);

	port_pin_set_output_level(MEAS_PIN, 1);

	delay_ms(10);
	adc_start_conversion(&adc_instance_app);
	while((adc_get_status(&adc_instance_app) & ADC_STATUS_RESULT_READY) != 1);
				
	adc_read(&adc_instance_app, &adc_result);

	port_pin_set_output_level(MEAS_PIN, 0);
				
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(HBAT_PIN, &pin_conf);
	port_pin_set_output_level(HBAT_PIN, 0);

	batt_mv = adc_result * 4900;
	batt_mv /= 4096;

	return (uint16_t)batt_mv;
}

static void app_wake(uint32_t sleptDuration)
{
	app_slept_duration = sleptDuration;
}

static uint32_t app_sleep(uint32_t sleepTime)
{
	uint32_t sleptTime = 0;
	
	if (isr_button || isr_vbus || isr_accel || (port_pin_get_input_level(USBVBUS_PIN) == USBVBUS_ACTIVE))
	{
		// USB connected or some interrupt came and we didn't finish doing whatever we had to. Don't sleep, but wait a while
		if (sleepTime > 10) sleepTime = 10;
		delay_ms(sleepTime);
		sleptTime = sleepTime;
	}
#ifdef MMC
	else if (mmc_flush_file)
	{
		// File needs to be written, no time to sleep now
		if (sleepTime > 10) sleepTime = 10;
		delay_ms(sleepTime);
		sleptTime = sleepTime;
	}
#endif	
	else
	{
		PMM_SleepReq_t sleepReq;
		sleepReq.sleepTimeMs = sleepTime;
		sleepReq.pmmWakeupCallback = app_wake;
		sleepReq.sleep_mode = SLEEP_MODE_STANDBY;
		app_slept_duration = 0;
		if (true == LORAWAN_ReadyToSleep(false))
		{
			HAL_RadioDeInit();
			SwTimerStop(digInputTimerId);
			if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
			{
			}
			PMM_Wakeup();
			sleptTime = app_slept_duration;
			HAL_Radio_resources_init();
			SwTimerStart(digInputTimerId, MS_TO_US(50), SW_TIMEOUT_RELATIVE, (void *)digInputTimerCb, NULL);
		}
	}
	
	if (sleptTime == 0)
	{
		// Oops, we didn't actually sleep. Let's make sure we don't crash the logic
		if (sleepTime > 10) sleepTime = 10;
		delay_ms(sleepTime);
		sleptTime = sleepTime;
	}
	
	return sleptTime;
}

P2PStatus_t app_p2p(void)
{
	P2PStatus_t retVal;
	retVal = P2P_ERROR;
	
	if (LORAWAN_CanSend() != LORAWAN_BUSY)
	{
		// We can switch to P2P mode
		if (LORAWAN_Pause() < 4200000000)
		{
			// Oops
			LORAWAN_Resume();
		}
		else
		{
			retVal = P2P_SUCCESS;
			p2pState = STATE_P2P_IDLE;
		}
	}
	return retVal;
}

P2PStatus_t app_lorawan(void)
{
	uint8_t paBoost;
	uint16_t preambleLen;
	uint32_t watchdogTimeout;

	if (STATE_P2P_IDLE != p2pState)
	{
		return P2P_ERROR;
	}
	
	if(loRa.featuresSupported & PA_SUPPORT)
	{
		paBoost = ENABLED;
	}
	else
	{
		paBoost = DISABLED;
	}
	preambleLen = RADIO_PHY_PREAMBLE_LENGTH;
	watchdogTimeout = RADIO_WATCHDOG_TIMEOUT;
	RADIO_SetAttr(PABOOST,(void *)&paBoost);
	RADIO_SetAttr(PREAMBLE_LEN,(void *)&preambleLen);
	RADIO_SetAttr(WATCHDOG_TIMEOUT,(void *)&watchdogTimeout);
	LORAWAN_Resume();
	
	return P2P_SUCCESS;
}


void app_p2p_radioConfig(P2PConfig_t *p2pConfig)
{
	uint16_t freq_hop_period;
	ecrConfig_t ecrConfig;
	uint8_t modulation;
	uint8_t bandwidth;
	uint8_t sf;
	uint8_t syncword;
	uint8_t ecr;
	uint8_t crcOn;

	sf = SF_12;
	bandwidth = BW_20KHZ;
	modulation = MODULATION_LORA;
	freq_hop_period = 0;
	syncword = 0x12;
	ecr = CR_4_5;
	crcOn = DISABLED;

	RADIO_SetAttr(CHANNEL_FREQUENCY,(void *)&(p2pConfig->frequency));
	RADIO_SetAttr(SPREADING_FACTOR,(void *)&sf);
	RADIO_SetAttr(BANDWIDTH,(void *)&bandwidth);
	RADIO_SetAttr(MODULATION,(void *)&modulation);
	RADIO_SetAttr(FREQUENCY_HOP_PERIOD,(void *)&freq_hop_period);
	RADIO_SetAttr(LORA_SYNC_WORD,(void *)&syncword);

	// Normally we'd need to restore old_ecr value back into radio before resuming MAC
	// But nobody ever changes ecr in the entire project and we're using the same value
	RADIO_SetAttr(ERROR_CODING_RATE,(void *)&ecr);

	RADIO_SetAttr(CRC_ON,(void *)&crcOn);
	RADIO_SetAttr(IQINVERTED,(void *)&(p2pConfig->iqInverted));
	RADIO_SetAttr(PREAMBLE_LEN,(void *)&(p2pConfig->preambleLen));

	RADIO_SetAttr(WATCHDOG_TIMEOUT,(void *)&(p2pConfig->watchdogTimeout));
}


void app_p2p_TimerCb(void *data)
{
	// Nothing..
}


P2PStatus_t app_p2p_tx(P2PConfig_t *p2pConfig)
{
	RadioReceiveParam_t RadioReceiveParam;
	RadioTransmitParam_t RadioTransmitParam;
	uint8_t paBoost;
	int8_t txPower;
	
	//paBoost = ENABLED;
	//txPower = 17;

	paBoost = DISABLED;
	txPower = 15;

	
	if (LORAWAN_MAC_PAUSED != LORAWAN_CanSend())
	{
		return P2P_ERROR;
	}
	
	// Check that the radio isn't doing stuff
	if (STATE_P2P_IDLE != p2pState)
	{
		return P2P_ERROR;
	}
		
	// Check duty cycle timer
	if (SwTimerIsRunning(p2pDcycleTimerId))
	{
		return P2P_DCYCLE_LIMIT;
	}
		
	/* Stop radio receive before transmission */
	RadioReceiveParam.action = RECEIVE_STOP;
	RADIO_Receive(&RadioReceiveParam);

	app_p2p_radioConfig(p2pConfig);

	RADIO_SetAttr(PABOOST,(void *)&paBoost);
	RADIO_SetAttr(OUTPUT_POWER,(void *)&txPower);
		
	RadioTransmitParam.bufferLen = p2pConfig->dataLen;
	RadioTransmitParam.bufferPtr = txBuffer;
		
	if (RADIO_ChannelActivityDetection() == 1)
	{
		// Channel has activity.
		return P2P_ERROR;
	}
		
	if (ERR_NONE == RADIO_Transmit(&RadioTransmitParam))
	{
		p2pState = STATE_P2P_TX;
		timeOnAir = SwTimerGetTime();
		return P2P_SUCCESS;
	}
	else
	{
		return P2P_ERROR;
	}
}


P2PStatus_t app_p2p_rx(P2PConfig_t *p2pConfig)
{
	RadioReceiveParam_t RadioReceiveParam;
	RadioTransmitParam_t RadioTransmitParam;
	uint8_t rxStart;
	
	if (LORAWAN_MAC_PAUSED != LORAWAN_CanSend())
	{
		return P2P_ERROR;
	}

	if (STATE_P2P_IDLE != p2pState)
	{
		return P2P_ERROR;
	}
	
	app_p2p_radioConfig(p2pConfig);

	rxStart = 1;
	if (1 == p2pConfig->doRxCad)
	{
		rxStart = RADIO_ChannelActivityDetection();
	}

	if (1 == rxStart)
	{
		// Channel has activity or we don't care
		RadioReceiveParam.action = RECEIVE_START;
		RadioReceiveParam.rxWindowSize = p2pConfig->rxWindowSize;
		if (ERR_NONE == RADIO_Receive(&RadioReceiveParam))
		{
			p2pState = STATE_P2P_RX;
			return P2P_SUCCESS;
		}
		else
		{
			return P2P_NO_DATA;
		}
	}
	else
	{
		return P2P_NO_DATA;
	}
}


P2PStatus_t app_p2p_tx_downlink(uint8_t dataLen)
{
	P2PConfig_t p2pConfig;

	p2pConfig.frequency = 869450000;
	p2pConfig.iqInverted = ENABLED;

	p2pConfig.preambleLen = 150;
	p2pConfig.watchdogTimeout = 37000;

	p2pConfig.dataLen = dataLen;
	
	return app_p2p_tx(&p2pConfig);
}


P2PStatus_t app_p2p_rx_downlink(void)
{
	P2PConfig_t p2pConfig;

	p2pConfig.frequency = 869450000;
	p2pConfig.iqInverted = ENABLED;

	p2pConfig.preambleLen = 150;
	p2pConfig.watchdogTimeout = 37000;

	p2pConfig.rxWindowSize = 12;
	p2pConfig.doRxCad = 1;
	
	return app_p2p_rx(&p2pConfig);
}


P2PStatus_t app_p2p_tx_uplink(uint8_t dataLen)
{
	P2PConfig_t p2pConfig;

	p2pConfig.frequency = 869550000;
	p2pConfig.iqInverted = DISABLED;

	p2pConfig.preambleLen = 12;
	p2pConfig.watchdogTimeout = 15000;

	p2pConfig.dataLen = dataLen;

	return app_p2p_tx(&p2pConfig);
}


P2PStatus_t app_p2p_rx_uplink(void)
{
	P2PConfig_t p2pConfig;

	p2pConfig.frequency = 869550000;
	p2pConfig.iqInverted = DISABLED;

	p2pConfig.preambleLen = 12;
	p2pConfig.watchdogTimeout = 120000;

	p2pConfig.rxWindowSize = 460;	// ~90 seconds
	p2pConfig.doRxCad = 0;

	return app_p2p_rx(&p2pConfig);
}


P2PStatus_t app_p2p_tx_cw(uint8_t on)
{
	if (on == 1)
	{
		P2PConfig_t p2pConfig;
		RadioReceiveParam_t RadioReceiveParam;
		RadioTransmitParam_t RadioTransmitParam;
		uint8_t paBoost;
		int8_t txPower;

		if (LORAWAN_MAC_PAUSED != LORAWAN_CanSend())
		{
			return P2P_ERROR;
		}
		
		// Check that the radio isn't doing stuff
		if (STATE_P2P_IDLE != p2pState)
		{
			return P2P_ERROR;
		}


		p2pConfig.frequency = 869450000;
		p2pConfig.iqInverted = ENABLED;

		p2pConfig.watchdogTimeout = 60000;
		
		//paBoost = ENABLED;
		//txPower = 17;

		paBoost = DISABLED;
		txPower = 15;
	
		/* Stop radio receive before transmission */
		RadioReceiveParam.action = RECEIVE_STOP;
		RADIO_Receive(&RadioReceiveParam);

		app_p2p_radioConfig(&p2pConfig);

		RADIO_SetAttr(PABOOST,(void *)&paBoost);
		RADIO_SetAttr(OUTPUT_POWER,(void *)&txPower);
	
		if (RADIO_ChannelActivityDetection() == 1)
		{
			// Channel has activity.
			return P2P_ERROR;
		}

		RADIO_TransmitCW();
		return P2P_SUCCESS;

	}
	else
	{
		RADIO_StopCW();
		return P2P_SUCCESS;
	}
}



#ifdef MMC
bool mmc_mount(void)
{
	FRESULT fr;
    FILINFO fno;
	char dir_name[16];
	uint16_t dir_number;
	bool dir_exists;

	fr = f_mount(&FatFs, "", 1);
	USB_sprintf(pstr1, "\r\nmount fr = %d\r\n", fr);
	
	dir_number = 0;
	dir_exists = false;
	do
	{
		sprintf(dir_name, "ACC_%04d", dir_number);

		fr = f_stat(dir_name, &fno);
		switch (fr)
		{
			case FR_OK:
				dir_exists = true;
				break;
			case FR_NO_FILE:
				dir_exists = false;
				break;
			default:
				USB_sprintf(pstr1, "\r\nf_stat: An error occurred. (%d)\r\n", fr);
				dir_exists = false;
				break;
		}
		dir_number++;
	} while (dir_exists);

    fr = f_mkdir(dir_name);
	USB_sprintf(pstr1, "\r\nmkdir %s, returned %d\r\n", dir_name, fr);

	fr = f_chdir(dir_name);	
	USB_sprintf(pstr1, "\r\nchdir %s, returned %d\r\n", dir_name, fr);

	acc_file = 0;

	return (FR_OK == fr);
}

bool mmc_unmount(void)
{
	FRESULT fr;
	fr = f_mount(NULL, "", 0);
	USB_sprintf(pstr1, "\r\nunmount fr = %d\r\n", fr);
	return (FR_OK == fr);
}

void mmc_write_file(void)
{
	UINT bw;
	FRESULT fr;
	char buf[128];
	uint16_t i, buf_len;

	mmc_flush_file = false;

	if (!mmc_mounted) return;

	sprintf(buf, "%08d.csv", acc_file);
	acc_file++;
	
	fr = f_open(&Fil, buf, FA_WRITE | FA_CREATE_ALWAYS);

	USB_sprintf(pstr1, "Writing file. f_open returned %d... ", fr);

	if (fr == FR_OK)
	{
		for (i = 0; i < acc_buffer*150; i++)
		{
			buf_len = sprintf(buf, "%d, %d, %d\r\n", acc_data_x[i], acc_data_y[i], acc_data_z[i]);
			fr = f_write(&Fil, buf, buf_len, &bw);
			if ((bw != buf_len) || (FR_OK != fr))
			{
				USB_sprintf(pstr1, "\r\nfile write error, %d, %d, %d\r\n", fr, buf_len, bw);
			}
		}
		
		fr = f_close(&Fil);
	}

	acc_buffer = 0;
	USB_printf("done\r\n");
}
#endif

void digInputTimerCb(void *data)
{
	// ctr values: 0 = armed for rising edge, 0xFF = action performed, waiting for falling edge, [0x01 - 0xFE] debouncing
	static uint8_t ctrButton = 0;

	// Always retrigger
	SwTimerStart(digInputTimerId, MS_TO_US(50), SW_TIMEOUT_RELATIVE, (void *)digInputTimerCb, NULL);
	
	// Handle SW0
	if (port_pin_get_input_level(SW0_PIN) == SW0_ACTIVE)
	{
		// Keep incrementing counter up to 1.5 seconds (30 * 50ms)
		if (ctrButton < 30)
		{
			ctrButton++;
		}
	}
	else
	{
		// Button released
		if (ctrButton == 30)
		{
			// Long press
			isr_button = false;
#ifdef MMC
			if (!mmc_mounted)
			{
				mmc_mounted = mmc_mount();
			}
			else
			{
				mmc_mounted = !mmc_unmount();
			}
			
			if (mmc_mounted)
			{
				LED_On(LED0_PIN);
				delay_ms(100);
				LED_Off(LED0_PIN);
				delay_ms(100);
				LED_On(LED0_PIN);
				delay_ms(200);
				LED_Off(LED0_PIN);
				delay_ms(200);
				LED_On(LED0_PIN);
				delay_ms(500);
				LED_Off(LED0_PIN);
			}
			else
			{
				LED_On(LED0_PIN);
				delay_ms(500);
				LED_Off(LED0_PIN);
				delay_ms(500);
				LED_On(LED0_PIN);
				delay_ms(200);
				LED_Off(LED0_PIN);
				delay_ms(200);
				LED_On(LED0_PIN);
				delay_ms(100);
				LED_Off(LED0_PIN);
			}
#endif
			
		}
		else if (ctrButton != 0)
		{
			// Click
			isr_button = false;
		}
		ctrButton = 0;
	}
	
	// Handle Vbus
	if (port_pin_get_input_level(USBVBUS_PIN) == USBVBUS_ACTIVE)
	{
		// Perform action just once
		if (!usb_power)
		{
			udc_start();
			usb_power = true;
		}
	}
	else
	{
		// Vbus disconnected
		if (usb_power)
		{
			udc_stop();
			usb_power = false;
			usb_connected = false; // just in case
		}
	}
	isr_vbus = false;	// We checked VBUS line one timer period (50ms) after ISR fired, we can go to sleep now if not connected

	// Handle INT from accelerometer
	if (port_pin_get_input_level(ACCINT_PIN) == ACCINT_ACTIVE)
	{
		isr_accel = false;
		spi_read_samples_accel();
	}
}

void gps_data_handler(void)
{
	static uint8_t c;
	static uint8_t u;
	u = gpsUpdate;
	while (sio2host_rx(&c, 1) != 0)
	{
		gps_parse(c);
		if (1 == nmeaEcho) USB_sprintf(pstr1, "%c", c);
	}
	if (gpsUpdate != u)
	{
#ifdef GPSTTFF
		gps_cold_boot();
		if (gpsRecords < sizeof(gpsRecord) / sizeof(gpsRecord_t))
		{
			gpsRecord[gpsRecords].lat = gpsLat;
			gpsRecord[gpsRecords].lon = gpsLon;
			gpsRecord[gpsRecords].tsBytes[0]  = (gpsYy & 0x7F) << 1;
			gpsRecord[gpsRecords].tsBytes[0] |= (gpsMo & 0x08) >> 3;
			gpsRecord[gpsRecords].tsBytes[1]  = (gpsMo & 0x07) << 5;
			gpsRecord[gpsRecords].tsBytes[1] |= (gpsDd & 0x1F);
			gpsRecord[gpsRecords].tsBytes[2]  = (gpsHh & 0x1F) << 3;
			gpsRecord[gpsRecords].tsBytes[2] |= (gpsMi & 0x38) >> 3;
			gpsRecord[gpsRecords].tsBytes[3]  = (gpsMi & 0x07) << 5;     // 3 LSB of minute
			gpsRecord[gpsRecords].tsBytes[3] |= (gpsSs & 0x3E) >> 1;
			gpsRecords++;
		}
#endif
		USB_sprintf(pstr1, "Lat = %d, Lon = %d, %02d:%02d:%02d, %02d.%02d.%02d\r\n", gpsLat, gpsLon, gpsHh, gpsMi, gpsSs, gpsDd, gpsMo, gpsYy);
	}
}


uint8_t parse_hex(uint8_t a)
{
	if (a >= '0' && a <= '9') return a - '0';
	if (a >= 'a' && a <= 'f') return a - 'a' + 10;
	if (a >= 'A' && a <= 'F') return a - 'A' + 10;
	return 0xFF;
}

void usb_data_handler(void)
{
	static uint8_t rxChar;
	static uint16_t count = 0;
	static uint64_t tStamp;
	static uint8_t bootloaderPassword[] = "bootload";
	static uint8_t killPassword[] = "kill";
	ChannelAttr_t ch_attr, ch_attr2;
	ChannelParameters_t ch_params;
	
	if (!usb_connected || !usb_power)
	{
		usbState = STATE_USB_IDLE;
	}

	if (usb_connected == true)
	{
		if(udi_cdc_is_rx_ready())
		{
			rxChar =  udi_cdc_getc();
			switch (usbState)
			{
				case STATE_USB_IDLE:
					switch (rxChar)
					{
						case '0':
							usbState = STATE_USB_BOOTLOADER;
							count = 0;
							break;
						case '1':
							usbState = STATE_USB_KILL;
							count = 0;
							break;
						case '3':
							i2c_init_temp();
							break;
						case '2':
							USB_sprintf(pstr1, "Temperature: %d\r\n", i2c_read_temp());
							break;
						case '4':
							app_p2p_tx_cw(1);
							break;	
						case '5':
							app_p2p_tx_cw(0);
							break;	
						case 'N':
							nmeaEcho ^= 1;
							break;
#ifdef GPSTTFF
						case 'x':
							gps_wake();
							gps_cold_boot();
							gpsRecords = 0;
							gpsNoSleep = 1;
							USB_printf("\r\nGPS module is reset\r\n");
							break;
						case 'y':
							USB_printf("\r\nlat,lon,ts\r\n");
							for (count = 0; count < gpsRecords; count++)
							{
								USB_sprintf(pstr1, "%d,%d,%lu\r\n", gpsRecord[count].lat, gpsRecord[count].lon, gpsRecord[count].ts);
							}
							break;
						case 'z':
							USB_printf("\r\nStop logging\r\n");
							gps_sleep();
							gpsNoSleep = 0;
							break;
#endif
						case 't':
							tStamp = SwTimerGetTime() / 1000;
							USB_sprintf(pstr1, "\r\ntStamp = %lu ms\r\n", tStamp);
							break;
						case 'a':
							USB_sprintf(pstr1, "\r\nVbat = %d mV\r\n", adc_read_vbat());
							break;
						case 'j':
							usbState = STATE_USB_JOINEUI;
							count = 0;
							break;
						case 'k':
							usbState = STATE_USB_APPKEY;
							count = 0;
							break;
						case 'i':
							appState = STATE_APP_NOT_INIT;
							break;
						case '-':
							usbState = STATE_USB_REMOVECHANNEL;
							break;
						case 's':
							USB_sprintf(pstr1, "\r\ntApp state = %d\r\n", appState);
							break;
						case 'd':
							read_deveui();
							USB_sprintf(pstr1, "\r\n%02X%02X%02X%02X%02X%02X%02X%02X\r\n", devEui[0], devEui[1], devEui[2], devEui[3], devEui[4], devEui[5], devEui[6], devEui[7]);
							break;
						case 'f':
							USB_sprintf(pstr1, "FEI = %d Hz\r\n", Radio_ReadFei());
							break;
						case 'Q':
							if (appState == STATE_APP_HIBERNATE)
							{
								USB_printf("\r\nSwitch to RX Uplinks\r\n");
								appState = STATE_APP_RX_UPLINKS;								
							}
							else if (appState == STATE_APP_RX_UPLINKS)
							{
								USB_printf("\r\nSwitch to Hibernate\r\n");
								appState = STATE_APP_HIBERNATE;
							}
							break;
						case 'p':
							if (LORAWAN_CanSend() != LORAWAN_MAC_PAUSED)
							{
								USB_printf("\r\nPausing MAC... ");
								USB_sprintf(pstr1, "app_p2p() returned %lu \r\n", app_p2p());
							}
							else
							{
								USB_printf("\r\Resuming MAC... ");
								USB_sprintf(pstr1, "app_lorawan() returned %lu \r\n", app_lorawan());
							}
							break;
						case 'T':
							for (count = 0; count < 12; count++)
							{
								txBuffer[count] = count;
							}
							USB_sprintf(pstr1, "app_p2p_tx_uplink() returned %d\r\n", app_p2p_tx_uplink(16));
							break;
						case 'R':
							USB_sprintf(pstr1, "app_p2p_rx_uplink() returned %d\r\n", app_p2p_rx_uplink());
							break;
						case 'G':
							for (count = 0; count < 20; count++)
							{
								txBuffer[count] = 19 - count;
							}
							USB_sprintf(pstr1, "app_p2p_tx_downlink() returned %d\r\n", app_p2p_tx_downlink(20));
							break;
						case 'F':
							USB_sprintf(pstr1, "app_p2p_rx_downlink() returned %d\r\n", app_p2p_rx_downlink());
							break;
						case 'L':
							USB_printf("\n");
							for (rxChar = 0; rxChar < 16; rxChar++)
							{
								uint8_t ch;
								ch = rxChar;
								LORAWAN_GetAttr(CH_PARAM_STATUS, &ch, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &ch, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &ch, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &ch, &ch_attr);
								if (ch_attr.status)
								{
									USB_sprintf(pstr1, "ch[%d]: f = %lu, drRange = %d, dutyCycle = %d\n", ch, ch_attr.frequency, ch_attr.dataRange, ch_attr.dutyCycle);
								}
							}
							break;
						case 'v':
							USB_printf("\nSwap channels: 9 <-> 5, 15 <-> 6\n");
							
							rxChar = 9;
							LORAWAN_GetAttr(CH_PARAM_STATUS, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &rxChar, &ch_attr);

							rxChar = 5;
							LORAWAN_GetAttr(CH_PARAM_STATUS, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &rxChar, &ch_attr2);

							
							ch_params.channelId = 5;
							ch_params.channelAttr.status = ch_attr.status;
							ch_params.channelAttr.frequency = ch_attr.frequency;
							ch_params.channelAttr.dutyCycle = ch_attr.dutyCycle;
							ch_params.channelAttr.dataRange = ch_attr.dataRange;
							LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);

							ch_params.channelId = 9;
							ch_params.channelAttr.status = ch_attr2.status;
							ch_params.channelAttr.frequency = ch_attr2.frequency;
							ch_params.channelAttr.dutyCycle = ch_attr2.dutyCycle;
							ch_params.channelAttr.dataRange = ch_attr2.dataRange;
							LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);

							// ----

							rxChar = 15;
							LORAWAN_GetAttr(CH_PARAM_STATUS, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &rxChar, &ch_attr);
							LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &rxChar, &ch_attr);
							
							rxChar = 6;
							LORAWAN_GetAttr(CH_PARAM_STATUS, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &rxChar, &ch_attr2);
							LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &rxChar, &ch_attr2);
							
							ch_params.channelId = 6;
							ch_params.channelAttr.status = ch_attr.status;
							ch_params.channelAttr.frequency = ch_attr.frequency;
							ch_params.channelAttr.dutyCycle = ch_attr.dutyCycle;
							ch_params.channelAttr.dataRange = ch_attr.dataRange;
							LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);

							ch_params.channelId = 15;
							ch_params.channelAttr.status = ch_attr2.status;
							ch_params.channelAttr.frequency = ch_attr2.frequency;
							ch_params.channelAttr.dutyCycle = ch_attr2.dutyCycle;
							ch_params.channelAttr.dataRange = ch_attr2.dataRange;
							LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
							LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);

							/*
							for (rxChar = 0; rxChar < 16; rxChar++)
							{
								if ((rxChar != 5) && (rxChar != 6))
								{
									ch_params.channelId = rxChar;
									ch_params.channelAttr.status = false;
									ch_params.channelAttr.frequency = 0;
									ch_params.channelAttr.dutyCycle = 0;
									ch_params.channelAttr.dataRange = 0;
									LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
									LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
									LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
									LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);
								}
							}
							*/
							break;
						case 'b':
							USB_printf("\Set everything to 865.7 MHz\n");
							for (rxChar = 3; rxChar < 16; rxChar++)
							{
								LORAWAN_GetAttr(CH_PARAM_STATUS, &rxChar, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_FREQUENCY, &rxChar, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_DR_RANGE, &rxChar, &ch_attr);
								LORAWAN_GetAttr(CH_PARAM_DUTYCYCLE, &rxChar, &ch_attr);

								ch_params.channelId = rxChar;
								ch_params.channelAttr.status = ch_attr.status;
								ch_params.channelAttr.frequency = 865700000;
								ch_params.channelAttr.dutyCycle = ch_attr.dutyCycle;
								ch_params.channelAttr.dataRange = ch_attr.dataRange;
								LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
								LORAWAN_SetAttr(CH_PARAM_FREQUENCY, &ch_params);
								LORAWAN_SetAttr(CH_PARAM_DR_RANGE, &ch_params);
								LORAWAN_SetAttr(CH_PARAM_DUTYCYCLE, &ch_params);
							}

							break;
#ifdef MMC
						case 'm':
							USB_sprintf(pstr1, "\r\mmc_flush_file == %d, mmc_mounted == %d, acc_buffer == %d\r\n", mmc_flush_file, mmc_mounted, acc_buffer);
							break;
						case 'f':
							mmc_write_file();
							break;
#endif
						default:
							USB_printf("\r\nUnknown command\r\n");
							break;						
					}
					break;
				case STATE_USB_JOINEUI:
					rxChar = parse_hex(rxChar);
					if (0xFF == rxChar)
					{
						USB_printf("\r\nError\r\n");
						usbState = STATE_USB_IDLE;
					}
					else
					{
						// Place received nibble at appropriate spot.
						// We'll receive 16 chars for 8 bytes
						if (0 == (count & 0x01))
						{
							txBuffer[count >> 1] = rxChar << 4;
						}
						else
						{
							txBuffer[count >> 1] |= rxChar;
						}
						count++;
						if (16 == count)
						{
							for (count = 0; count < 8; count++)
							{
								joinEui[count] = txBuffer[count];
								txBuffer[count] = 0;
							}
							USB_printf("\r\nUpdated JoinEUI\r\n");
							usbState = STATE_USB_IDLE;
						}
					}
					break;
				case STATE_USB_APPKEY:
					rxChar = parse_hex(rxChar);
					if (0xFF == rxChar)
					{
						USB_printf("\r\nError\r\n");
						usbState = STATE_USB_IDLE;
					}
					else
					{
						// Place received nibble at appropriate spot.
						// We'll receive 32 chars for 16 bytes
						if (0 == (count & 0x01))
						{
							txBuffer[count >> 1] = rxChar << 4;
						}
						else
						{
							txBuffer[count >> 1] |= rxChar;
						}
						count++;
						if (32 == count)
						{
							for (count = 0; count < 16; count++)
							{
								appKey[count] = txBuffer[count];
								txBuffer[count] = 0;
							}
							USB_printf("\r\nUpdated AppKey\r\n");
							usbState = STATE_USB_IDLE;
						}
					}
					break;
				case STATE_USB_REMOVECHANNEL:
					rxChar = parse_hex(rxChar);
					if (0xFF == rxChar)
					{
						USB_printf("\r\nError\r\n");
						usbState = STATE_USB_IDLE;
					}
					else
					{
						USB_sprintf(pstr1, "\r\nRemoving channel %d\r\n", rxChar);
						ch_params.channelId = rxChar;
						ch_params.channelAttr.status = false;
						LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
						usbState = STATE_USB_IDLE;
					}
					break;
				case STATE_USB_BOOTLOADER:
					if (rxChar == bootloaderPassword[count])
					{
						count++;
					}
					else
					{
						USB_printf("\r\nBack to app\r\n");
						usbState = STATE_USB_IDLE;
					}
					
					if (count == (sizeof(bootloaderPassword) - 1))
					{
						reset_to_bootloader();
					}
					break;
				case STATE_USB_KILL:
					if (rxChar == killPassword[count])
					{
						count++;
					}
					else
					{
						USB_printf("\r\nBack to app\r\n");
						usbState = STATE_USB_IDLE;
					}
					
					if (count == (sizeof(killPassword) - 1))
					{
						for (count = 0; count < 16; count++)
						{
							appKey[count] = 0xFF;
						}
						NVIC_SystemReset();
					}
					break;
				default:
					usbState = STATE_USB_IDLE;
					break;
			}
		}
	}
}



void app_init(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	// Outputs
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin_conf);
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSEN_PIN, &pin_conf);
	port_pin_set_output_level(GPSEN_PIN, GPSEN_INACTIVE);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ACCPWR_PIN, &pin_conf);
	port_pin_set_output_level(ACCPWR_PIN, 0);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ACCCS_PIN, &pin_conf);
	port_pin_set_output_level(ACCCS_PIN, 0);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(MEAS_PIN, &pin_conf);
	port_pin_set_output_level(MEAS_PIN, MEAS_INACTIVE);

	// Other application pins set to low power
	/*
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(MOSI_PIN, &pin_conf);
	port_pin_set_output_level(MOSI_PIN, 0);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SCLK_PIN, &pin_conf);
	port_pin_set_output_level(SCLK_PIN, 0);
	*/
	
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(HBAT_PIN, &pin_conf);
	port_pin_set_output_level(HBAT_PIN, 0);

	// Unused pins, output low
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA22, &pin_conf);
	port_pin_set_output_level(PIN_PA22, 0);
	
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SWDCLK_PIN, &pin_conf);
	port_pin_set_output_level(SWDCLK_PIN, 0);
	
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SWDIO_PIN, &pin_conf);
	port_pin_set_output_level(SWDIO_PIN, 0);

	// Inputs
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(SW0_PIN, &pin_conf);
	
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(CHG_PIN, &pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ACCINT_PIN, &pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(USBVBUS_PIN, &pin_conf);

/*
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(MISO_PIN, &pin_conf);
*/
	
	usb_power = false;		// If USB is actually plugged in digInputTimerCb will switch flag on and turn USB peripheral on
	
	// Application timers
	SwTimerCreate(&digInputTimerId);
	SwTimerStart(digInputTimerId, MS_TO_US(50), SW_TIMEOUT_RELATIVE, (void *)digInputTimerCb, NULL);

	SwTimerCreate(&ledTimerId);

	SwTimerCreate(&appTimerId);
	SwTimerCreate(&wdTimerId);

	SwTimerCreate(&p2pDcycleTimerId);

	// Peripherals
	i2c_init_temp();
	i2c_read_temp();	// this leaves the sensor in shutdown state
	gps_sleep();

	delay_ms(10);		// wait before powering the ADXL362 up a bit to make sure it starts from 0V. Datasheet "Power Supply Requirements"
	port_pin_set_output_level(ACCPWR_PIN, 1);
	port_pin_set_output_level(ACCCS_PIN, 1);
	spi_init_accel();
	
	adc_init_vbat();
	
	// Start app in a known state
	app_slept_duration = 0;
	//txInterval = 600000;		// 10 minutes
	txInterval = 120000;		// 2 minutes
	txIntervalLocate = 120000;	// 2 minutes
	
    gpsLat = -1000;
    gpsLon = -1000;
    gpsHh = 0;
    gpsMi = 0;
    gpsSs = 0;
    gpsDd = 0;
    gpsMo = 0;
    gpsYy = 0;
    gpsUpdate = 0;
	gpsHdop = 65000;
	
	appState = STATE_APP_NOT_INIT;
	gpsState = STATE_GPS_OFF;
	gpsN = 0;
	gpsI = 0;
	nmeaEcho = 1;

#ifdef GPSTTFF
	gpsNoSleep = 0;
#endif

	txCounter = 0;

	stepsCounter = 0;
	lastTxTs = SwTimerGetTime();
	
#ifdef MMC
	mmc_mounted = false;
	acc_buffer = 0;
	acc_file = 0;
	mmc_flush_file = false;
#endif	
	
}

void gps_sleep(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	sio2host_deinit();

	// GPS off
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSEN_PIN, &pin_conf);
	port_pin_set_output_level(GPSEN_PIN, GPSEN_INACTIVE);

	delay_ms(1);

	// GPS UART & RST set to low power mode
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSTX_PIN, &pin_conf);
	port_pin_set_output_level(GPSTX_PIN, 0);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSRX_PIN, &pin_conf);
	port_pin_set_output_level(GPSRX_PIN, 0);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSRST_PIN, &pin_conf);
	port_pin_set_output_level(GPSRST_PIN, 0);
	
}

void gps_wake(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	// GPS on
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSEN_PIN, &pin_conf);
	port_pin_set_output_level(GPSEN_PIN, GPSEN_ACTIVE);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GPSRST_PIN, &pin_conf);
	port_pin_set_output_level(GPSRST_PIN, 1);

	sio2host_init();
}

#ifdef GPSTTFF
void gps_cold_boot(void)
{
	uint8_t gpsReset[] = "$PMTK104*37\r\n";
	port_pin_set_output_level(GPSRST_PIN, 0);
	delay_ms(500);
	port_pin_set_output_level(GPSRST_PIN, 1);
	delay_ms(500);
	sio2host_tx(gpsReset, sizeof(gpsReset) - 1);
}
#endif


StackRetStatus_t app_init_lora(void)
{
	StackRetStatus_t status = LORAWAN_SUCCESS;
	JoinNonceType_t nonceType = JOIN_NONCE_RANDOM;
	EdClass_t class = CLASS_A;
	IsmBand_t ismBand = ISM_EU868;
	bool adr = false;
	uint8_t dr, power, i;
	bool appKeyOk, joinEuiOk;

	read_deveui();

	LORAWAN_Init(app_tx_callback, app_join_callback);
	
	if (LORAWAN_SUCCESS == status) status = LORAWAN_Reset(ismBand);

	if ((ismBand == ISM_NA915) || (ismBand == ISM_AU915))
	{
		#define MAX_NA_CHANNELS 72
		#define MAX_SUBBAND_CHANNELS 8

		ChannelParameters_t ch_params;
		uint8_t allowed_min_125khz_ch,allowed_max_125khz_ch,allowed_500khz_channel;
		allowed_min_125khz_ch = (SUBBAND-1)*MAX_SUBBAND_CHANNELS;
		allowed_max_125khz_ch = ((SUBBAND-1)*MAX_SUBBAND_CHANNELS) + 7 ;
		allowed_500khz_channel = SUBBAND+63;
		for (ch_params.channelId = 0; ch_params.channelId < MAX_NA_CHANNELS; ch_params.channelId++)
		{
			if((ch_params.channelId >= allowed_min_125khz_ch) && (ch_params.channelId <= allowed_max_125khz_ch))
			{
				ch_params.channelAttr.status = true;
			}
			else if(ch_params.channelId == allowed_500khz_channel)
			{
				ch_params.channelAttr.status = true;
			}
			else
			{
				ch_params.channelAttr.status = false;
			}

			if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(CH_PARAM_STATUS, &ch_params);
		}
	}

	if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(JOIN_NONCE_TYPE, &nonceType);
	if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(EDCLASS, &class);
	if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(ADR, &adr);

	LORAWAN_GetAttr(CURRENT_DATARATE, 0, &dr);

	// DR0
	if (dr != DR0)
	{
		dr = DR0;
		LORAWAN_SetAttr(CURRENT_DATARATE, &dr);
		USB_printf("Set DR to SF12BW125\r\n");
	}
	
	appKeyOk = false;
	for (i = 0; i < 16; i++)
	{
		if (appKey[i] != 0xFF) appKeyOk = true;
	}

	joinEuiOk = false;
	for (i = 0; i < 8; i++)
	{
		if (joinEui[i] != 0xFF) joinEuiOk = true;
	}

	if (appKeyOk && joinEuiOk)
	{
		if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(DEV_EUI, devEui);
		if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(JOIN_EUI, joinEui);
		if (LORAWAN_SUCCESS == status) status = LORAWAN_SetAttr(APP_KEY, appKey);
	}
	else
	{
		status = LORAWAN_KEYS_NOT_INITIALIZED;
	}


	return status;

	/*
    status = PDS_IsRestorable();
    if(status)
    {
        PDS_RestoreAll();
		...
        LORAWAN_GetAttr(ISMBAND,NULL,&prevBand);
	}
	//PDS_DeleteAll();
	*/
}


// LED blink timer call back handler
void ledTimerCb(void *data)
{
	static bool ledOn = false;

	if (ledOn)
	{
		LED_Off(LED0_PIN);
		ledOn = false;
		SwTimerStart(ledTimerId,MS_TO_US(200),SW_TIMEOUT_RELATIVE,(void *)ledTimerCb,NULL);
	}
	else
	{
		LED_On(LED0_PIN);
		ledOn = true;
		SwTimerStart(ledTimerId,MS_TO_US(50),SW_TIMEOUT_RELATIVE,(void *)ledTimerCb,NULL);
	}
}

static void ledBlink(void)
{
	SwTimerStart(ledTimerId,MS_TO_US(10),SW_TIMEOUT_RELATIVE,(void *)ledTimerCb,NULL);
}

static void ledStop(void)
{
	SwTimerStop(ledTimerId);
	LED_Off(LED0_PIN);
}


static void read_deveui(void)
{
	uint8_t i = 0, j = 0;
	uint8_t invalidMODULEDevEui[8];
	uint8_t moduleDevEUI[8];
	for (i = 0; i < 8; i += 2, j++)
	{
		moduleDevEUI[i] = (NVM_UID_ADDRESS[j] & 0xFF);
		moduleDevEUI[i + 1] = (NVM_UID_ADDRESS[j] >> 8);
	}
	memset(&invalidMODULEDevEui, 0xFF, sizeof(invalidMODULEDevEui));
	// If Module doesnot have DEV EUI, the read value will be of all 0xFF,
	// Set devEUI in conf_app.h in that case
	if(0 != memcmp(&moduleDevEUI, &invalidMODULEDevEui, sizeof(devEui)))
	{
		// Set EUI addr in Module if there
		memcpy(devEui, moduleDevEUI, sizeof(devEui));
	}
}


void app_join_callback(StackRetStatus_t status)
{
	int8_t ch_ID;

	SwTimerStop(wdTimerId);

	if(LORAWAN_SUCCESS == status)
	{
		USB_printf("a");
		appState = STATE_APP_SLEEP;
		sleepInterval = 5000; // First packet in 5 seconds

		#if (EU_BAND == 1)
		// if EU 868mhz region then limit channel selections to mandatory 125khz channels 0,1 and 2
		uint8_t band;
		LORAWAN_GetAttr(ISMBAND,NULL,&band);

		if (band == ISM_EU868)
		{
			UpdateChId_t update_chid;
			#define MAX_EU_CHANNELS 16

			uint8_t channelIndex;
			for (channelIndex = 0; channelIndex < MAX_EU_CHANNELS; channelIndex++)
			{
				// set channel number using update_chid.channelIndex
				update_chid.channelIndex = channelIndex;
				if(channelIndex < 3)
				{
					// Enable or Disable the channel using update_chid.statusNew
					update_chid.statusNew = ENABLED;
				}
				else
				{
					update_chid.statusNew = DISABLED;
				}

				//Call LORAREG_SetAttr CHANNEL_ID_STATUS to set the new channel parameters
				LORAREG_SetAttr (CHANNEL_ID_STATUS,&update_chid);
			}
		}
		#endif

		PDS_StoreAll();
	}
	else
	{
		USB_printf("d");
		sleepInterval = 10000;
		appState = STATE_APP_SLEEP_JOIN;
	}
}


static void app_rx_data(void *appHandle, appCbParams_t *appdata)
{
	uint8_t *pData = appdata->param.rxData.pData;
	uint8_t dataLength = appdata->param.rxData.dataLength;

	//Successful reception
	if((dataLength > 0U) && (NULL != pData))
	{
		if ((dataLength == 3) || (dataLength == 4))
		{
			// 0 - GPS off
			// 1 - GPS on for x packets
			// 2 - GPS on every x packets
			gpsState = pData[2];

			if ((gpsState == STATE_GPS_OFF) && (dataLength == 3))
			{
				// update interval in multiples of 2 minutes. Only valid for GPS off
				txInterval = (uint32_t)(pData[1] + 1) * 2 * 60 * 1000;
			}
			else if ((gpsState == STATE_GPS_ON_FOR) && (dataLength == 4))
			{
				gpsN = 0;
				gpsI = pData[3];
				// update interval in multiples of 2 minutes. Only valid for GPS ON/EVERY
				txIntervalLocate = (uint32_t)(pData[1] + 1) * 2 * 60 * 1000;
			}
			else if ((gpsState == STATE_GPS_ON_EVERY) && (dataLength == 4))
			{
				gpsN = pData[3];
				gpsI = 0;
				// update interval in multiples of 2 minutes. Only valid for GPS ON/EVERY
				txIntervalLocate = (uint32_t)(pData[1] + 1) * 2 * 60 * 1000;
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
				sleepInterval = 900000;
			}
		}
	}
}

void app_tx_callback(void *appHandle, appCbParams_t *appdata)
{
	if (LORAWAN_CanSend() != LORAWAN_MAC_PAUSED)
	{
		//LoRaWAN Rx
		USB_printf("r");
		LED_Off(LED0_PIN);
	
		// Stop watchdog timer
		SwTimerStop(wdTimerId);

		if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
		{
			if (LORAWAN_SUCCESS == appdata->param.rxData.status)
			{
				app_rx_data(appHandle, appdata);
			}
			else
			{
				USB_printf("x");
			}
		}
		else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
		{
			if (LORAWAN_SUCCESS != appdata->param.transCmpl.status)
			{
				USB_printf("X");
			}
		}
	
		appState = STATE_APP_SLEEP;

		// Application-configured interval until next Tx
		if (STATE_GPS_OFF == gpsState) sleepInterval = txInterval;
		else sleepInterval = txIntervalLocate;
	}
	else
	{
		// LoRa Rx
		USB_printf("R");

		AppState_t newState;

		if (STATE_P2P_TX == p2pState)
		{
			timeOnAir = SwTimerGetTime() - timeOnAir;
			// Start duty cycle timer to respect 10% DC limit
			USB_sprintf(pstr1, "\r\ntimeOnAir = %lu ms\r\n", timeOnAir / 1000);
			SwTimerStart(p2pDcycleTimerId, timeOnAir * 9, SW_TIMEOUT_RELATIVE, (void *)app_p2p_TimerCb, NULL);
		}
		p2pState = STATE_P2P_IDLE;

		newState = appState;
		
		if (STATE_APP_P2P_GPS_TX == appState)
		{
			newState = STATE_APP_P2P_GPS;
		}

		if (STATE_APP_P2P_WAIT_DOWNLINK == appState)
		{
			// Just in case no other branch takes us anywhere
			newState = STATE_APP_SLEEP_2;
		}

		if (LORAWAN_EVT_RX_DATA_AVAILABLE == appdata->evt)
		{
			if (LORAWAN_RADIO_SUCCESS == appdata->param.rxData.status)
			{
				uint8_t i;
				uint8_t *pData = appdata->param.rxData.pData;
				uint8_t dataLength = appdata->param.rxData.dataLength;
				USB_printf("\r\nRx data: ");
				for (i = 0; i < dataLength; i++) USB_sprintf(pstr1, "%02X ", pData[i]);
				USB_printf("\r\n");
				
				if (STATE_APP_P2P_WAIT_DOWNLINK == appState)
				{
					newState = STATE_APP_P2P_GPS;
				}
			}
			else
			{
				USB_printf("x");
				USB_sprintf(pstr1, "app_tx_callback() appdata->param.rxData.status = %d\r\n", appdata->param.rxData.status);
			}
		}
		else if(LORAWAN_EVT_TRANSACTION_COMPLETE == appdata->evt)
		{
			if (LORAWAN_RADIO_NO_DATA == appdata->param.transCmpl.status)
			{
				USB_printf("\r\nDone, no data\r\n");
			}
			else if (LORAWAN_RADIO_SUCCESS != appdata->param.transCmpl.status)
			{
				USB_printf("X");
				USB_sprintf(pstr1, "app_tx_callback() appdata->param.transCmpl.status = %d\r\n", appdata->param.transCmpl.status);
			}
		}
		
		appState = newState;		
	}
	
}


void gpsTimerCallback(void)
{
	USB_printf("c");
	appState = STATE_APP_GPS_OFF;
}

void wdTimerCallback(void)
{
	USB_printf("q");
	LED_Off(LED0_PIN);
	// No clue what to do. Reset MAC, for lack of a better idea
	appState = STATE_APP_NOT_INIT;
	gpsState = STATE_GPS_OFF;
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
		val = batt_mv * 168;
		val -= 581500;
		val /= 1000;
		return (uint8_t) val;
	}
}

void app_prepare_buffer_lorawan()
{
	uint16_t dt;
	
	txBuffer[0] = 0x80 + gpsState;

	txBuffer[1] = txCounter >> 8;
	txBuffer[2] = txCounter & 0xFF;

	txBuffer[3] = battery_percent(adc_read_vbat());
	
	txBuffer[4] = i2c_read_temp();
	
	// Timestamp bit tetris
	// 7 bits: Year
	// 4 bits: Month
	// 5 bits: Day
	// 5 bits: Hour
	// 6 bits: Minute
	// 5 bits remaining
	txBuffer[5]  = (gpsYy & 0x7F) << 1;     // 7 bits of year
	txBuffer[5] |= (gpsMo & 0x08) >> 3;     // and 1 MSB of month
	txBuffer[6]  = (gpsMo & 0x07) << 5;     // 3 LSB of month
	txBuffer[6] |= (gpsDd & 0x1F);          // 5 bits of day
	txBuffer[7]  = (gpsHh & 0x1F) << 3;     // 5 bits of hour
	txBuffer[7] |= (gpsMi & 0x38) >> 3;     // 3 MSB of minute
	txBuffer[8]  = (gpsMi & 0x07) << 5;     // 3 LSB of minute

	txBuffer[9] = (gpsLat >> 24) & 0xFF;
	txBuffer[10] = (gpsLat >> 16) & 0xFF;
	txBuffer[11] = (gpsLat >> 8) & 0xFF;
	txBuffer[12] = gpsLat & 0xFF;

	txBuffer[13] = (gpsLon >> 24) & 0xFF;
	txBuffer[14] = (gpsLon >> 16) & 0xFF;
	txBuffer[15] = (gpsLon >> 8) & 0xFF;
	txBuffer[16] = gpsLon & 0xFF;
	
	dt = (uint16_t)((SwTimerGetTime() - lastTxTs) / 1000000);
	txBuffer[17] = (dt >> 8) & 0xFF;
	txBuffer[18] = dt & 0xFF;

	txBuffer[19] = (gpsHdop >> 8) & 0xFF;
	txBuffer[20] = gpsHdop & 0xFF;

/*
	txBuffer[19] = (stepsCounter >> 24) & 0xFF;
	txBuffer[20] = (stepsCounter >> 16) & 0xFF;
	txBuffer[21] = (stepsCounter >> 8) & 0xFF;
	txBuffer[22] = stepsCounter & 0xFF;
*/

	//stepsCounter = 0;
	lastTxTs = SwTimerGetTime();
}

void app_prepare_buffer_p2p()
{
	txBuffer[0] = txCounter >> 8;
	txBuffer[1] = txCounter & 0xFF;

	// Timestamp bit tetris
	// 7 bits: Year
	// 4 bits: Month
	// 5 bits: Day
	// 5 bits: Hour
	// 6 bits: Minute
	// 5 bits remaining
	txBuffer[2]  = (gpsYy & 0x7F) << 1;     // 7 bits of year
	txBuffer[2] |= (gpsMo & 0x08) >> 3;     // and 1 MSB of month
	txBuffer[3]  = (gpsMo & 0x07) << 5;     // 3 LSB of month
	txBuffer[3] |= (gpsDd & 0x1F);          // 5 bits of day
	txBuffer[4]  = (gpsHh & 0x1F) << 3;     // 5 bits of hour
	txBuffer[4] |= (gpsMi & 0x38) >> 3;     // 3 MSB of minute
	txBuffer[5]  = (gpsMi & 0x07) << 5;     // 3 LSB of minute

	txBuffer[6] = (gpsLat >> 24) & 0xFF;
	txBuffer[7] = (gpsLat >> 16) & 0xFF;
	txBuffer[8] = (gpsLat >> 8) & 0xFF;
	txBuffer[9] = gpsLat & 0xFF;

	txBuffer[10] = (gpsLon >> 24) & 0xFF;
	txBuffer[11] = (gpsLon >> 16) & 0xFF;
	txBuffer[12] = (gpsLon >> 8) & 0xFF;
	txBuffer[13] = gpsLon & 0xFF;
	
	txBuffer[14] = (gpsHdop >> 8) & 0xFF;
	txBuffer[15] = gpsHdop & 0xFF;
}


StackRetStatus_t app_tx()
{
	static StackRetStatus_t status;
	static bool adr;
	static uint8_t dr;
    static uint8_t avail_payload;

	LORAWAN_GetAttr(ADR, 0, &adr);
	if (adr)
	{
		adr = false;
		LORAWAN_SetAttr(ADR, &adr);
		USB_printf("Set ADR off\r\n");
	}
	
	// DR0
	LORAWAN_GetAttr(CURRENT_DATARATE, 0, &dr);
	if (dr != MAC_DATARATE_MAX_EU)
	{
		dr = MAC_DATARATE_MAX_EU;
		LORAWAN_SetAttr(CURRENT_DATARATE, &dr);
		USB_printf("Set DR to min\r\n");
	}


    lorawanSendReq.buffer = txBuffer;
    lorawanSendReq.bufferLength = 21;
    lorawanSendReq.confirmed = LORAWAN_UNCNF;
    lorawanSendReq.port = 10;
    LORAWAN_GetAttr(NEXT_PAYLOAD_SIZE, NULL, &avail_payload);
    if (avail_payload < lorawanSendReq.bufferLength)
    {
	    // At DR0 for NA and AU regions Max payload = 3 bytes or less, due to FHDR(7) and FPORT(1) byte
	    USB_sprintf(pstr1, "\r\nSending %d bytes of payload - DR limitation\r\n", avail_payload);
	    lorawanSendReq.bufferLength = avail_payload;
    }

	if (LORAWAN_CanSend() == LORAWAN_SUCCESS)
	{
		// Now that we're pretty sure we can Tx, prepare the buffer
		app_prepare_buffer_lorawan();
		status = LORAWAN_Send(&lorawanSendReq);
		if (LORAWAN_SUCCESS == status)
		{
			USB_printf("t");
			txCounter++;
		}
	}
	else
	{
		status = LORAWAN_NO_CHANNELS_FOUND;
	}
	
	return status;
}


void app_run()
{
	static StackRetStatus_t status;
	static int32_t sleepIntervalFull = 0;
	
#ifdef MMC
	// We may need to flush the buffers to file, but make sure the LoRaWAN stack is not busy first.
	// Otherwise we might be taking too long to write the file and mess stuff up
	static LorawanStatus_t macStatus;
	LORAWAN_GetAttr(LORAWAN_STATUS, NULL, &macStatus.value);
	if (mmc_flush_file && (0 == macStatus.macState))
	{
		// Ok to write file
		mmc_write_file();
	}
#endif
	
	switch (appState)
	{
		case STATE_APP_NOT_INIT:
			USB_printf("I");
			if (LORAWAN_SUCCESS == app_init_lora())
			{
				// we have LoRaWAN credentials
				appState = STATE_APP_NOT_JOINED;
			}
			else
			{
				appState = STATE_APP_HIBERNATE;
			}
			break;
		case STATE_APP_NOT_JOINED:
			USB_printf("J");
			if (LORAWAN_SUCCESS == LORAWAN_Join(LORAWAN_OTAA))
			{
				USB_printf("j");
				appState = STATE_APP_JOINING;
				SwTimerStart(wdTimerId, MS_TO_US(15000), SW_TIMEOUT_RELATIVE, (void *)wdTimerCallback, NULL);
			}
			else
			{
				sleepInterval = 10000;
				appState = STATE_APP_SLEEP_JOIN;
			}
			break;
		case STATE_APP_SLEEP_JOIN:
			// Stay here until sleep interval is exhausted
			sleepInterval -= app_sleep(sleepInterval);
			if (sleepInterval <= 0)
			{
				appState = STATE_APP_NOT_JOINED;
				USB_printf("w");
			}
			break;
		case STATE_APP_JOINING:
			// exit this state through app_join_callback
			break;
		case STATE_APP_SLEEP:
			sleepIntervalFull = sleepInterval;
			if (sleepIntervalFull > 28000)
			{
				sleepInterval = 28000;
			}
			appState = STATE_APP_SLEEP_1;
			break;
		case STATE_APP_SLEEP_1:
			// Stay here until sleep interval is exhausted
			sleepInterval -= app_sleep(sleepInterval);
			if (sleepInterval <= 0)
			{
				sleepInterval = 28000;
				sleepIntervalFull -= 28000;
				if (sleepIntervalFull <= 0)
				{
					appState = STATE_APP_WAKE;
					USB_printf("w");
				}
				else
				{
					appState = STATE_APP_SLEEP_2;

					// Check for downlink
					if (P2P_SUCCESS == app_p2p())
					{

#if 0
						// Do this regardless of any reception
						USB_printf("+");
						appState = STATE_APP_P2P_GPS;;
						p2pStopTime = SwTimerGetTime() + 1800000000UL;		// 30 minutes from now
#else
						if (P2P_SUCCESS == app_p2p_rx_downlink())
						{
							// We may be receiving a packet. Have to wait
							// for callback to figure it out
							USB_printf("+");
							appState = STATE_APP_P2P_WAIT_DOWNLINK;
							p2pStopTime = SwTimerGetTime() + 1800000000UL;		// 30 minutes from now
						}
						else
						{
							USB_printf("-");
						}
#endif
					}
				}
			}
			break;
		case STATE_APP_SLEEP_2:
			// Return to SLEEP state, go back to LoRaWAN mode
			if (P2P_SUCCESS != app_lorawan())
			{
				USB_printf("!");
			}
			sleepInterval = sleepIntervalFull;
			appState = STATE_APP_SLEEP;
			break;
		case STATE_APP_WAKE:
			USB_printf("W");
			// Can we Tx? If not, no sense of trying to,
			// especially if we plan on turning GPS on
			if (LORAWAN_SUCCESS == LORAWAN_CanSend())
			{
				USB_printf("^");
				ledBlink();
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
							SwTimerStart(appTimerId, MS_TO_US(600000), SW_TIMEOUT_RELATIVE, (void *)gpsTimerCallback, NULL);
							gpsUpdate = 0;
							gps_wake();
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
							SwTimerStart(appTimerId, MS_TO_US(600000), SW_TIMEOUT_RELATIVE, (void *)gpsTimerCallback, NULL);
							gpsUpdate = 0;
							gps_wake();
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
				// We can't Tx right now, sleep some more. Try in 30 seconds
				USB_printf("v");
				USB_sprintf(pstr1, "LORAWAN_CanSend() returned %d\r\n", LORAWAN_CanSend());
				appState = STATE_APP_SLEEP;
				sleepInterval = 30000;
			}
			break;
		case STATE_APP_GPS:
			// Can also exit this state through gpsTimerCallback callback
			if (gpsUpdate == 5)
			{
				appState = STATE_APP_GPS_OFF;
				SwTimerStop(appTimerId);
			}
			break;
		case STATE_APP_GPS_OFF:
			USB_printf("O");
			gps_sleep();
			appState = STATE_APP_TX;
			break;
		case STATE_APP_TX:
			USB_printf("T");
			ledStop();
			LED_On(LED0_PIN);
			appState = STATE_APP_TX_WAIT;
			status = app_tx();
			SwTimerStart(wdTimerId, MS_TO_US(15000), SW_TIMEOUT_RELATIVE, (void *)wdTimerCallback, NULL);
			switch (status)
			{
				case LORAWAN_SUCCESS:
					break;
				case LORAWAN_NO_CHANNELS_FOUND:
					// Try again in 30s
					appState = STATE_APP_SLEEP;
					sleepInterval = 30000;
					SwTimerStop(wdTimerId);
					break;
				case LORAWAN_NWK_NOT_JOINED:
				case LORAWAN_FCNTR_ERROR_REJOIN_NEEDED:
					USB_printf("Z");
					appState = STATE_APP_NOT_JOINED;
					SwTimerStop(wdTimerId);
					break;
				default:
					USB_printf("X");
					appState = STATE_APP_NOT_INIT;
					SwTimerStop(wdTimerId);
					break;
			}
			break;
		case STATE_APP_TX_WAIT:
			// exit this state through app_tx_callback
			break;
		case STATE_APP_HIBERNATE:
			// Stay in low power mode. Interrupts (such as USB)
			// still wake system up and can kick it into gear
#ifdef GPSTTFF
			if (gpsNoSleep == 0)
			{
#endif
				app_sleep(3600000);
#ifdef GPSTTFF
			}
#endif
			break;
		case STATE_APP_P2P_WAIT_DOWNLINK:
			// exit this state through app_tx_callback
			break;
		case STATE_APP_P2P_GPS:
			if (p2pStopTime > SwTimerGetTime())
			{
				gps_wake();
				USB_printf("u");
				// now start Tx'ing as fast as allowable by DC constraints
				sleepInterval = 5000;
				appState = STATE_APP_P2P_GPS_WAIT;
			}
			else
			{
				// Done, stop GPS and return to sleep
				gps_sleep();
				sleepInterval = 5000;
				appState = STATE_APP_SLEEP_2;
			}
			break;
		case STATE_APP_P2P_GPS_WAIT:
			// Stay here until sleep interval is exhausted
			sleepInterval -= app_sleep(sleepInterval);
			if (sleepInterval <= 0)
			{
				app_prepare_buffer_p2p();
				if (P2P_SUCCESS == app_p2p_tx_uplink(16))
				{
					txCounter++;
					appState = STATE_APP_P2P_GPS_TX;
					USB_printf("U");
				}
				else
				{
					appState = STATE_APP_P2P_GPS;
				}
			}
			break;
		case STATE_APP_P2P_GPS_TX:
			// exit this state through app_tx_callback
			break;
		case STATE_APP_RX_UPLINKS:
			// Keep listening for uplinks
			app_p2p_rx_uplink();
			if (STATE_P2P_RX == p2pState)
			{
				int32_t fei;
				fei = Radio_ReadFei();
				if (fei != 0) USB_sprintf(pstr1, "FEI = %d Hz\r\n", fei);
			}
			app_sleep(500);
			break;

	}
}



static void reset_to_bootloader(void)
{
	*DBL_TAP_PTR = DBL_TAP_MAGIC;
	NVIC_SystemReset();
}


/* eof  */
