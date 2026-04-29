/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        WFG120
#define MANUFACTURER_ID   WWF

// ---------- Core buses ----------
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define USE_I2C
#define USE_I2C_DEVICE_1

#define USE_ADC

// ---------- IMU (SPI1) ----------
#define USE_GYRO
#define USE_ACC

// 允许你三选一/多选自动探测：ICM42605 / ICM42688P / BMI270
#define USE_GYRO_SPI_ICM42605
#define USE_ACC_SPI_ICM42605

#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P

// BMI270 在 BF 里是 ACCGYRO 驱动宏（不是 USE_GYRO_SPI_BMI270 / USE_ACC_SPI_BMI270）
#define USE_ACCGYRO_BMI270
#define USE_ACCGYRO_W688
#define USE_ACCGYRO_W689

#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_CS_PIN        PB2
#define GYRO_1_EXTI_PIN      PA4
#define GYRO_1_ALIGN         CW270_DEG
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_2_SPI_INSTANCE SPI2
#define GYRO_2_CS_PIN        PB12
#define GYRO_2_EXTI_PIN      PD10
#define GYRO_2_ALIGN         CW270_DEG
#define ENSURE_MPU_DATA_READY_IS_LOW

// ---------- Baro / Mag (I2C1) ----------
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE I2CDEV_2

#define USE_MAG
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE I2CDEV_2
// QMC5883 常见地址 0x0D -> 13；不是默认地址时需要显式设置，否则可能不工作
#define MAG_I2C_ADDRESS 13

// ---------- OSD (MAX7456 on SPI3) ----------
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI3
#define MAX7456_SPI_CS_PIN   PD7

// ---------- SDCARD ----------
#define USE_SDCARD
#define SDIO_DEVICE SDIODEV_1
#define SDIO_USE_4BIT 1
#define SDCARD_DETECT_PIN NONE
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD

// ---------- GPS ----------
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_NMEA
#define USE_GPS_AUTO_BAUD
#define USE_GPS_RESCUE
#define GPS_UART SERIAL_PORT_USART3

// ---------- RX ----------
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define SERIALRX_UART SERIAL_PORT_UART6

// ---------- Pins ----------
#define BEEPER_PIN           PD11
#define BEEPER_INVERTED

#define LED0_PIN             PE2
#define LED1_PIN             PE3
#define LED2_PIN             PE4
#define LED_STRIP_PIN        PA15

// UART pins
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define UART3_TX_PIN         PD8
#define UART3_RX_PIN         PD9
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1
#define UART5_TX_PIN         PB5
#define UART5_RX_PIN         PB6
#define UART6_TX_PIN         PC6
#define UART6_RX_PIN         PC7
#define UART7_TX_PIN         PE8
#define UART7_RX_PIN         PE7
#define UART8_TX_PIN         PE1
#define UART8_RX_PIN         PE0

// I2C1 pins
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9

#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11

// SPI pins
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7

#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15

#define SPI3_SCK_PIN         PB3
#define SPI3_SDI_PIN         PB4
#define SPI3_SDO_PIN         PD6

#define SPI4_SCK_PIN         PE12
#define SPI4_SDI_PIN         PE13
#define SPI4_SDO_PIN         PE14

// SD pins
#define SDIO_CMD_PIN         PD2
#define SDIO_CK_PIN          PC12
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11

// ADC
#define ADC_VBAT_PIN         PC0
#define ADC_CURR_PIN         PC1
//#define ADC_RSSI_PIN         PB1

// PINIO
#define USE_PINIO
#define PINIO1_PIN           PC2
#define PINIO2_PIN           PC3
#define PINIO1_BOX           40
#define PINIO2_BOX           41

// ---------- Motors / Servos (8 motors + 2 servos) ----------
#define MOTOR1_PIN           PD12
#define MOTOR2_PIN           PD13
#define MOTOR3_PIN           PD14
#define MOTOR4_PIN           PD15
#define MOTOR5_PIN           PE5
#define MOTOR6_PIN           PE6
#define MOTOR7_PIN           PE9
#define MOTOR8_PIN           PE11
#define MOTOR9_PIN           PB0
#define MOTOR10_PIN          PB1

// Timer map: 关键点是“第 3 参数”要选对复用(AF)分组；DMA 先用 -1（不强依赖 DSHOT_DMAR）
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PD12 , 1,  0) \
    TIMER_PIN_MAP( 1, PD13 , 1,  0) \
    TIMER_PIN_MAP( 2, PD14 , 1,  0) \
    TIMER_PIN_MAP( 3, PD15 , 1, -1) \
    TIMER_PIN_MAP( 4, PE5  , 1, -1) \
    TIMER_PIN_MAP( 5, PE6  , 1, -1) \
    TIMER_PIN_MAP( 6, PE9  , 1,  0) \
    TIMER_PIN_MAP( 7, PE11 , 1,  1) \
    TIMER_PIN_MAP( 8, PB0  , 2,  0) \
    TIMER_PIN_MAP( 9, PB1  , 2,  0)

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE  250

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF  // 禁用DMA DSHOT
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_ON  // 开启位冲击

#define SYSTEM_HSE_MHZ 8
