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

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        WFG001A
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
#define GYRO_1_CS_PIN        PA4
#define GYRO_1_EXTI_PIN      PB2
#define GYRO_1_ALIGN         CW90_DEG
#define ENSURE_MPU_DATA_READY_IS_LOW

// ---------- Baro / Mag (I2C1) ----------
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE I2CDEV_1

#define USE_MAG
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE I2CDEV_1
// QMC5883 常见地址 0x0D -> 13；不是默认地址时需要显式设置，否则可能不工作
#define MAG_I2C_ADDRESS 13

// ---------- OSD (MAX7456 on SPI3) ----------
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI3
#define MAX7456_SPI_CS_PIN   PB0

// ---------- Blackbox Flash (W25Q128 on SPI2) ----------
#define USE_FLASH
#define USE_FLASH_SPI
#define USE_FLASH_W25Q128FV
#define FLASH_SPI_INSTANCE SPI2
#define FLASH_CS_PIN       PB12
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH

// ---------- GPS ----------
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_NMEA
#define USE_GPS_AUTO_BAUD
#define USE_GPS_RESCUE
#define GPS_UART SERIAL_PORT_USART3

// ---------- RX ----------
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define SERIALRX_UART SERIAL_PORT_UART2

// ---------- Pins ----------
#define BEEPER_PIN           PC13
#define BEEPER_INVERTED

#define LED0_PIN             PC14
#define LED1_PIN             PC15
#define LED_STRIP_PIN        PA8

// UART pins
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define UART3_TX_PIN         PB10
#define UART3_RX_PIN         PB11
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1
#define UART5_TX_PIN         PC12
#define UART5_RX_PIN         PD2

// I2C1 pins
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9

// SPI pins
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7

#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PC2
#define SPI2_SDO_PIN         PC3

#define SPI3_SCK_PIN         PC10
#define SPI3_SDI_PIN         PC11
#define SPI3_SDO_PIN         PB5

// ADC
#define ADC_VBAT_PIN         PC0
#define ADC_CURR_PIN         PC1
#define ADC_RSSI_PIN         PB1

// PINIO
#define USE_PINIO
#define PINIO1_PIN           PD10
#define PINIO2_PIN           PD11
#define PINIO1_BOX           40
#define PINIO2_BOX           41

// ---------- Motors / Servos (8 motors + 2 servos) ----------
#define MOTOR1_PIN           PB14
#define MOTOR2_PIN           PB15
#define MOTOR3_PIN           PC8
#define MOTOR4_PIN           PC9
#define MOTOR5_PIN           PB3
#define MOTOR6_PIN           PB4
#define MOTOR7_PIN           PB6
#define MOTOR8_PIN           PB7

#define SERVO1_PIN           PC6
#define SERVO2_PIN           PC7

// Timer map: 关键点是“第 3 参数”要选对复用(AF)分组；DMA 先用 -1（不强依赖 DSHOT_DMAR）
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB14, 3, -1) \
    TIMER_PIN_MAP( 1, PB15, 3, -1) \
    TIMER_PIN_MAP( 2, PC8 , 1,  0) \
    TIMER_PIN_MAP( 3, PC9 , 1,  0) \
    TIMER_PIN_MAP( 4, PB3 , 1,  0) \
    TIMER_PIN_MAP( 5, PB4 , 1,  0) \
    TIMER_PIN_MAP( 6, PB6 , 1,  0) \
    TIMER_PIN_MAP( 7, PB7 , 1,  0) \
    TIMER_PIN_MAP( 8, PC6 , 2,  0) \
    TIMER_PIN_MAP( 9, PC7 , 2,  1)

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE  250

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF  // 禁用DMA DSHOT
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_ON  // 开启位冲击

#define SYSTEM_HSE_MHZ 8
