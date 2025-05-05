/**
 * \file
 *
 * \brief WLR089 Module Xplained Pro board definition
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
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
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

#ifndef WLR089_XPLAINED_PRO_H_INCLUDED
#define WLR089_XPLAINED_PRO_H_INCLUDED

#include <conf_board.h>
#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup group_common_boards
 * \defgroup samr34_xplained_pro_group SAM R34 Xplained Pro board
 *
 * @{
 */

void system_board_init(void);

/**
 * \defgroup samr34xplained_pro_features_group Features
 *
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

/** Name string macro */
#define BOARD_NAME                "WLR089_XPLAINED_PRO"

/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS  0 /* Not Mounted */
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625
/** @} */

/** \name LED0 definitions
 *  @{ */
#define LED0_PIN                  PIN_PA19
#define LED0_ACTIVE               false
#define LED0_INACTIVE             !LED0_ACTIVE
/** @} */



#define GPSRST_PIN                PIN_PA27
#define GPSTX_PIN                 PIN_PA08
#define GPSRX_PIN				  PIN_PA05

#define GPSEN_PIN                 PIN_PA14
#define GPSEN_ACTIVE              true
#define GPSEN_INACTIVE            !GPSEN_ACTIVE

#define ACCPWR_PIN                PIN_PA18

#define ACCCS_PIN                 PIN_PA23

#define MEAS_PIN                  PIN_PA28
#define MEAS_ACTIVE               true
#define MEAS_INACTIVE             !MEAS_ACTIVE

#define HBAT_PIN                  PIN_PB03
#define MOSI_PIN                  PIN_PB22
#define MISO_PIN                  PIN_PB02
#define SCLK_PIN                  PIN_PB23

#define SWDCLK_PIN                PIN_PA30
#define SWDIO_PIN                 PIN_PA31


/** \name SW0 definitions
 *  @{ */
#define SW0_PIN                   PIN_PA07
#define SW0_ACTIVE                false
#define SW0_INACTIVE              !SW0_ACTIVE
#define SW0_EIC_PIN               PIN_PA07A_EIC_EXTINT7
#define SW0_EIC_MUX               MUX_PA07A_EIC_EXTINT7
#define SW0_EIC_PINMUX            PINMUX_PA07A_EIC_EXTINT7
#define SW0_EIC_LINE              7
/** @} */

#define CHG_PIN                   PIN_PA15
#define CHG_ACTIVE                false
#define CHG_INACTIVE              !CHG_ACTIVE
#define CHG_EIC_PIN               PIN_PA15A_EIC_EXTINT15
#define CHG_EIC_MUX               MUX_PA15A_EIC_EXTINT15
#define CHG_EIC_PINMUX            PINMUX_PA15A_EIC_EXTINT15
#define CHG_EIC_LINE              15

#define ACCINT_PIN                PIN_PA04
#define ACCINT_ACTIVE             false
#define ACCINT_INACTIVE           !ACCINT_ACTIVE
#define ACCINT_EIC_PIN            PIN_PA04A_EIC_EXTINT4
#define ACCINT_EIC_MUX            MUX_PA04A_EIC_EXTINT4
#define ACCINT_EIC_PINMUX         PINMUX_PA04A_EIC_EXTINT4
#define ACCINT_EIC_LINE           4

#define USBVBUS_PIN               PIN_PA06
#define USBVBUS_ACTIVE            true
#define USBVBUS_INACTIVE          !USBVBUS_ACTIVE
#define USBVBUS_EIC_PIN           PIN_PA06A_EIC_EXTINT6
#define USBVBUS_EIC_MUX           MUX_PA06A_EIC_EXTINT6
#define USBVBUS_EIC_PINMUX        PINMUX_PA06A_EIC_EXTINT6
#define USBVBUS_EIC_LINE          6

#define HBAT_ADC_MODULE           ADC
#define HBAT_ADC_CHANNEL          11
#define HBAT_ADC_PIN              PIN_PB03B_ADC_AIN11
#define HBAT_ADC_MUX              MUX_PB03B_ADC_AIN11
#define HBAT_ADC_PINMUX           PINMUX_PB03B_ADC_AIN11



/** \name RF SWITCH definitions
 *  @{ */
#define RF_SWITCH_PIN				PIN_PA13
#define RF_SWITCH_ACTIVE             true
#define RF_SWITCH_INACTIVE	        !RF_SWITCH_ACTIVE
/** @} */

/** \name TCXO PWR Pin definitions
 *  @{ */
#define TCXO_PWR_PIN				PIN_PA09
#define TCXO_PWR_ACTIVE             true
#define TCXO_PWR_INACTIVE	        !TCXO_PWR_ACTIVE
/** @} */


#define LED0_PWM3CTRL_MODULE     TCC0
#define LED0_PWM3CTRL_CHANNEL    3
#define LED0_PWM3CTRL_OUTPUT     3
#define LED0_PWM3CTRL_PIN        PIN_PA19F_TCC0_WO3
#define LED0_PWM3CTRL_MUX        MUX_PA19F_TCC0_WO3
#define LED0_PWM3CTRL_PINMUX     PINMUX_PA19F_TCC0_WO3
/** @} */


/** Number of on-board LEDs */
#define LED_COUNT                 1



/** \name USB definitions
 * @{
 */
#define USB_ID
#define USB_TARGET_DP_PIN            PIN_PA25G_USB_DP
#define USB_TARGET_DP_MUX            MUX_PA25G_USB_DP
#define USB_TARGET_DP_PINMUX         PINMUX_PA25G_USB_DP
#define USB_TARGET_DM_PIN            PIN_PA24G_USB_DM
#define USB_TARGET_DM_MUX            MUX_PA24G_USB_DM
#define USB_TARGET_DM_PINMUX         PINMUX_PA24G_USB_DM


#define SX_RF_SPI                  SERCOM4
#define SX_RF_RESET_PIN            PIN_PB15
#define SX_RF_SPI_CS               PIN_PB31
#define SX_RF_SPI_MOSI             PIN_PB30
#define SX_RF_SPI_MISO             PIN_PC19
#define SX_RF_SPI_SCK              PIN_PC18

#define SX_RF_SPI_SERCOM_MUX_SETTING   SPI_SIGNAL_MUX_SETTING_E
#define SX_RF_SPI_SERCOM_PINMUX_PAD0   PINMUX_PC19F_SERCOM4_PAD0
#define SX_RF_SPI_SERCOM_PINMUX_PAD1   PINMUX_UNUSED
#define SX_RF_SPI_SERCOM_PINMUX_PAD2   PINMUX_PB30F_SERCOM4_PAD2
#define SX_RF_SPI_SERCOM_PINMUX_PAD3   PINMUX_PC18F_SERCOM4_PAD3

#define DIO0_PIN                   PIN_PB16
#define DIO0_ACTIVE                true
#define DIO0_INACTIVE              !DIO0_ACTIVE
#define DIO0_EIC_PIN               PIN_PB16A_EIC_EXTINT0
#define DIO0_EIC_MUX               MUX_PB16A_EIC_EXTINT0
#define DIO0_EIC_PINMUX            PINMUX_PB16A_EIC_EXTINT0
#define DIO0_EIC_LINE              0 

#define DIO1_PIN                   PIN_PA11
#define DIO1_ACTIVE                true
#define DIO1_INACTIVE              !DIO1_ACTIVE
#define DIO1_EIC_PIN               PIN_PA11A_EIC_EXTINT11
#define DIO1_EIC_MUX               MUX_PA11A_EIC_EXTINT11
#define DIO1_EIC_PINMUX            PINMUX_PA11A_EIC_EXTINT11
#define DIO1_EIC_LINE              11

#define DIO2_PIN                   PIN_PA12
#define DIO2_ACTIVE                true
#define DIO2_INACTIVE              !DIO2_ACTIVE
#define DIO2_EIC_PIN               PIN_PA12A_EIC_EXTINT12
#define DIO2_EIC_MUX               MUX_PA12A_EIC_EXTINT12
#define DIO2_EIC_PINMUX            PINMUX_PA12A_EIC_EXTINT12
#define DIO2_EIC_LINE              12

#define DIO3_PIN                   PIN_PB17
#define DIO3_ACTIVE                true
#define DIO3_INACTIVE              !DIO3_ACTIVE
#define DIO3_EIC_PIN               PIN_PB17A_EIC_EXTINT1
#define DIO3_EIC_MUX               MUX_PB17A_EIC_EXTINT1
#define DIO3_EIC_PINMUX            PINMUX_PB17A_EIC_EXTINT1
#define DIO3_EIC_LINE              1

#define DIO4_PIN                   PIN_PA10
#define DIO4_ACTIVE                true
#define DIO4_INACTIVE              !DIO4_ACTIVE
#define DIO4_EIC_PIN               PIN_PA10A_EIC_EXTINT10
#define DIO4_EIC_MUX               MUX_PA10A_EIC_EXTINT10
#define DIO4_EIC_PINMUX            PINMUX_PA10A_EIC_EXTINT10
#define DIO4_EIC_LINE              10

#define DIO5_PIN                   PIN_PB00
#define DIO5_ACTIVE                true
#define DIO5_INACTIVE              !DIO5_ACTIVE
#define DIO5_EIC_PIN               PIN_PB00A_EIC_EXTINT0
#define DIO5_EIC_MUX               MUX_PB00A_EIC_EXTINT0
#define DIO5_EIC_PINMUX            PINMUX_PB00A_EIC_EXTINT0
#define DIO5_EIC_LINE              0

#define SX_RF_RESET_HIGH		   true
#define SX_RF_RESET_LOW		       !SX_RF_RESET_HIGH

/** @} */

/**
 * \brief Turns off the specified LEDs.
 *
 * \param led_gpio LED to turn off (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Off(led_gpio)     port_pin_set_output_level(led_gpio,true)

/**
 * \brief Turns on the specified LEDs.
 *
 * \param led_gpio LED to turn on (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_On(led_gpio)      port_pin_set_output_level(led_gpio,false)

/**
 * \brief Toggles the specified LEDs.
 *
 * \param led_gpio LED to toggle (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Toggle(led_gpio)  port_pin_toggle_output_level(led_gpio)

/** @} */

#ifdef __cplusplus
}
#endif

#endif  /* SAMR34_XPLAINED_PRO_H_INCLUDED */
