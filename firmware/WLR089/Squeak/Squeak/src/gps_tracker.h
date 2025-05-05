/**
* \file  enddevice_demo.h
*
* \brief Getting Started LoRaWAN [USB_CDC] Demo Application
*
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef GPS_TRACKER_H_
#define GPS_TRACKER_H_


/*
#define CONF_I2C_MASTER_MODULE    SERCOM5
//#define CONF_MASTER_SDA_PINMUX    PINMUX_PB02D_SERCOM5_PAD0
//#define CONF_MASTER_SCK_PINMUX    PINMUX_PA23D_SERCOM5_PAD1
#define CONF_MASTER_SDA_PINMUX    PINMUX_PB16C_SERCOM5_PAD0
#define CONF_MASTER_SCK_PINMUX    PINMUX_PB17C_SERCOM5_PAD1
*/


/*********************************************************************//**
 \brief      Macro to check the LoRaWAN stack status attribute
             network joined bit
*************************************************************************/
#define LORAWAN_NW_JOINED	0x01

typedef enum _AppState_t
{
	STATE_APP_NOT_INIT,
	STATE_APP_NOT_JOINED,
	STATE_APP_SLEEP_JOIN,
	STATE_APP_JOINING,
	STATE_APP_SLEEP,
	STATE_APP_SLEEP_1,
	STATE_APP_SLEEP_2,
	STATE_APP_WAKE,
	STATE_APP_GPS,
	STATE_APP_GPS_OFF,
	STATE_APP_TX,
	STATE_APP_TX_WAIT,
	STATE_APP_HIBERNATE,
	STATE_APP_P2P_WAIT_DOWNLINK,
	STATE_APP_P2P_GPS,
	STATE_APP_P2P_GPS_WAIT,
	STATE_APP_P2P_GPS_TX,
	STATE_APP_RX_UPLINKS
}AppState_t;


typedef enum _GpsState_t
{
	STATE_GPS_OFF,
	STATE_GPS_ON_FOR,
	STATE_GPS_ON_EVERY
}GpsState_t;

typedef enum _UsbState_t
{
	STATE_USB_IDLE,
	STATE_USB_JOINEUI,
	STATE_USB_APPKEY,
	STATE_USB_BOOTLOADER,
	STATE_USB_KILL,
	STATE_USB_REMOVECHANNEL
}UsbState_t;

typedef enum _P2PState_t
{
	STATE_P2P_IDLE,
	STATE_P2P_TX,
	STATE_P2P_RX
}P2PState_t;

typedef enum _P2PStatus_t
{
	P2P_SUCCESS,
	P2P_ERROR,
	P2P_NO_DATA,
	P2P_DCYCLE_LIMIT
}P2PStatus_t;

typedef struct  
{
	uint32_t frequency;
	uint32_t watchdogTimeout;
	uint16_t preambleLen;
	uint16_t rxWindowSize;
	uint8_t iqInverted;
	uint8_t dataLen;
	uint8_t doRxCad;
}P2PConfig_t;

void app_init(void);
void app_run(void);

void usb_data_handler(void);
void gps_data_handler(void);

#endif /* GPS_TRACKER_H_ */

