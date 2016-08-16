/*
* This file is part of Cleanflight.
*
* Cleanflight is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Cleanflight is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#define TARGET_BOARD_IDENTIFIER "PXR4"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#define USBD_PRODUCT_STRING     "PixRacer"

#define USE_EXTI

#define LED0                    PB11    //red
#define LED1                    PB3     //blue
#define LED2                    PB1     //green

#define BEEPER                  PA15
#define BEEPER_INVERTED

#define INVERTER                PC13
#define INVERTER_USART          USART1

#define MPU9250_CS_PIN          PC2
#define MPU9250_SPI_INSTANCE    SPI1

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN       CW180_DEG_FLIP

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN      CW180_DEG_FLIP

#define MAG
#define USE_MAG_AK8963

//#define BARO
//#define USE_BARO_MS5611
//#define MS5611_I2C_INSTANCE     I2CDEV_1

#define USE_SDCARD

#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PD2

// SPI is on the APB2 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

/* ***PR wonders - stock from Bluejay target *** */
#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

#define USABLE_TIMER_CHANNEL_COUNT 7

// MPU9250 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define MPU_INT_EXTI PA6

#define USE_VCP
#define VBUS_SENSING_PIN PA9
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7     // RX is used only (SerialRX)
#define UART6_TX_PIN            PC6

/*
#define USE_UART7
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            PE8

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1
*/

#define SERIAL_PORT_COUNT       6 //VCP, UART1, UART2, UART3, UART4

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC2
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define I2C_DEVICE              (I2CDEV_1)
#define USE_I2C_PULLUP
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9


#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define VBAT_ADC_PIN            PA2
#define CURRENT_METER_ADC_PIN   PA3
#define RSSI_ADC_PIN            PC1


#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        FEATURE_BLACKBOX

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff

#define USED_TIMERS             ( TIM_N(1) | TIM_N(3) | TIM_N(4))