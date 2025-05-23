/**
* \file  conf_app.h
*
* \brief Getting Started LoRAWAN [USB_CDC] Demo Application
*
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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


#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/****************************** INCLUDES **************************************/

/****************************** MACROS **************************************/

// For resetting to bootloader from application
#define HSRAM_ADDR            _UL_(0x20000000) /**< HSRAM base address */
#define HSRAM_SIZE            _UL_(0x00008000) /* 32 kB */
#define DBL_TAP_PTR ((volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4))
#define DBL_TAP_MAGIC 0xf01669ef // Randomly selected, adjusted to have first and last bit set


/*Define the Sub band of Channels to be enabled by default for the application*/
/* Only necessary for regions that have defined subbands like NA915 and AU915 */
#define SUBBAND 1
#if ((SUBBAND < 1 ) || (SUBBAND > 8 ) )
#error " Invalid Value of Subband"
#endif

#define NVM_UID_ADDRESS   ((volatile uint16_t *)(0x0080400AU))

/* OTAA Join Parameters */
#define DEVICE_EUI								{0x00, 0x04, 0xA3, 0x0B, 0x00, 0x1A, 0xD9, 0x82}	// stolen from breadboard-1
#define APPLICATION_EUI							{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define APPLICATION_KEY							{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

/*
#define DEVICE_EUI								{0x00, 0x04, 0xA3, 0x0B, 0x00, 0x1A, 0xD9, 0x82}	// stolen from breadboard-1
#define APPLICATION_EUI							{0x60, 0x81, 0xF9, 0x68, 0x26, 0xC6, 0x7C, 0x1A}
#define APPLICATION_KEY							{0xCC, 0xE3, 0x9F, 0x47, 0x2B, 0xBE, 0x84, 0x68, 0x22, 0xD7, 0x8D, 0x44, 0xBE, 0x63, 0x24, 0x00}
*/

#endif /* APP_CONFIG_H_ */