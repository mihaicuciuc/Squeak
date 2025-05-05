/**
* \file  main.c
*
* \brief Getting Started  [USB_CDC] Demo Application main file
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

/****************************** INCLUDES **************************************/

#include <stdarg.h>
#include "system_low_power.h"
#include "radio_driver_hal.h"
#include "lorawan.h"
#include "sys.h"
#include "system_init.h"
#include "system_assert.h"
#include "aes_engine.h"
#include "gps_tracker.h"
#include "sio2host.h"
#include "extint.h"
#include "conf_app.h"
#include "sw_timer.h"
#include "sleep_timer.h"

#include "conf_sio2host.h"
#include "pds_interface.h"

#include "conf_usb.h"


/************************** Global variables ***********************************/

volatile bool usb_connected;
volatile bool isr_button;
volatile bool isr_vbus;
volatile bool isr_accel;

/************************** Function Prototypes ********************************/

int USB_sprintf(char * str, const char * format, ... ) __attribute__ ((format (gnu_printf, 2, 3)));
void USB_printf(const char* str);


static void isr_button_callback(void);
static void isr_vbus_callback(void);
static void isr_accel_callback(void);


static void isr_button_callback(void)
{
	volatile uint8_t a;
	a = port_pin_get_input_level(SW0_PIN);
	isr_button = true;
}

static void isr_vbus_callback(void)
{
	volatile uint8_t a;
	a = port_pin_get_input_level(USBVBUS_PIN);
	isr_vbus = true;
}

static void isr_accel_callback(void)
{
	volatile uint8_t a;
	a = port_pin_get_input_level(ACCINT_PIN);
	isr_accel = true;
}

static void configure_eic_callbacks(void)
{
	extint_register_callback(isr_button_callback, SW0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(SW0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);

	extint_register_callback(isr_vbus_callback, USBVBUS_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(USBVBUS_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);

	extint_register_callback(isr_accel_callback, ACCINT_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(ACCINT_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}


static void configure_extints(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = SW0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = SW0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.gpio_pin_pull      = EXTINT_PULL_NONE;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(SW0_EIC_LINE, &eint_chan_conf);
	
	eint_chan_conf.gpio_pin           = USBVBUS_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = USBVBUS_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_RISING;
	eint_chan_conf.gpio_pin_pull      = EXTINT_PULL_NONE;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(USBVBUS_EIC_LINE, &eint_chan_conf);
	
	eint_chan_conf.gpio_pin           = ACCINT_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = ACCINT_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.gpio_pin_pull      = EXTINT_PULL_NONE;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(ACCINT_EIC_LINE, &eint_chan_conf);
}


int main(void)
{
    /* System Initialization */
    system_init();
    /* Initialize the delay driver */
    delay_init();
    /* Initialize the board target resources */
    board_init();

    INTERRUPT_GlobalInterruptEnable();

    /* Initialize the Radio Hardware */
    HAL_RadioInit();
    /* Initialize the AES Hardware Engine */
    AESInit();
    /* Initialize the Software Timer Module */
    SystemTimerInit();
    /* Initialize the Sleep Timer Module */
    SleepTimerInit();
    /* PDS Module Init */
    PDS_Init();

    Stack_Init();

	USB_printf("\r\n***********************************************\r\n");

	usb_connected = false;
	isr_button = false;
	isr_vbus = false;
	isr_accel = false;

    app_init();

	configure_extints();
	configure_eic_callbacks();

    while (1)
    {
        SYSTEM_RunTasks();
        usb_data_handler();
		gps_data_handler();
		app_run();
    }
}


void main_callback_suspend_action(void)
{
	// Add code here
}

void main_callback_resume_action(void)
{
	// Add code here
}

void main_callback_sof_action(void)
{
	if (!usb_connected)
	return;
	//Add code here
}


bool main_cdc_enable(uint8_t port)
{
	// Add code here
	return true;
}

void main_cdc_disable(uint8_t port)
{
	// Close communication
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable)
	{
		// Host terminal has open COM
		usb_connected = true;
	}
	else
	{
		// Host terminal has close COM
		usb_connected = false;
	}
}


void USB_printf(const char* str)
{
	if (usb_connected)
	{
		udi_cdc_multi_write_buf(1,str,strlen(str));
	}
}

int USB_sprintf(char * str, const char * format, ... )
{
	if (usb_connected)
	{
		va_list args;
		va_start(args, format);
		vsprintf(str,format, args);
		va_end(args);
		udi_cdc_multi_write_buf(1,str,strlen(str));
	}
	return 0;
}



/**
 End of File
 */
