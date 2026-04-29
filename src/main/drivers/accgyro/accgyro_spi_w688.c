/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef USE_ACCGYRO_W688

#include "common/axis.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_w688.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "sensors/acceleration.h" // 包含 GRAVITY_MSS
#include "sensors/gyro_init.h" 

// W688 supports up to 14 MHz SPI clock (datasheet Sec 16.2.4)
#define W688_MAX_SPI_CLK_HZ         10000000

// Use datasheet-real ODR names here. In 6DOF mode the device runs from the
// gyroscope natural frequency, so "8K/4K/2K/1K" style names are misleading.
typedef enum {
    W688_REG_WHO_AM_I = 0x00,
    W688_REG_CTRL1 = 0x02,
    W688_REG_CTRL2 = 0x03,
    W688_REG_CTRL3 = 0x04,
    W688_REG_CTRL5 = 0x06,
    W688_REG_CTRL7 = 0x08,
    W688_REG_CTRL9 = 0x0A,
    W688_REG_STATUSINT = 0x2D,
    W688_REG_STATUS0 = 0x2E,
    W688_REG_TEMP_L = 0x33,
    W688_REG_TEMP_H = 0x34,
    W688_REG_AX_L = 0x35,
    W688_REG_AX_H = 0x36,
    W688_REG_AY_L = 0x37,
    W688_REG_AY_H = 0x38,
    W688_REG_AZ_L = 0x39,
    W688_REG_AZ_H = 0x3A,
    W688_REG_GX_L = 0x3B,
    W688_REG_GX_H = 0x3C,
    W688_REG_GY_L = 0x3D,
    W688_REG_GY_H = 0x3E,
    W688_REG_GZ_L = 0x3F,
    W688_REG_GZ_H = 0x40,
    W688_REG_COD_STATUS = 0x46,
    W688_REG_RESET_DONE = 0x4D,
    W688_REG_RESET = 0x60,
} w688Register_e;

typedef enum {
    W688_DISABLE_ALL = 0x00,
    W688_ACC_ENABLE = 0x01,
    W688_GYR_ENABLE = 0x02,
    W688_ACCGYR_ENABLE = W688_ACC_ENABLE | W688_GYR_ENABLE,
} w688SensorEnable_e;

typedef enum {
    W688_ACC_RANGE_2G = 0x00 << 4,
    W688_ACC_RANGE_4G = 0x01 << 4,
    W688_ACC_RANGE_8G = 0x02 << 4,
    W688_ACC_RANGE_16G = 0x03 << 4,
} w688AccRange_e;

typedef enum {
    W688_ACC_ODR_7174HZ = 0x00,     // 6DOF only
    W688_ACC_ODR_3587HZ = 0x01,     // 6DOF only
    W688_ACC_ODR_1793HZ = 0x02,     // 6DOF only
    W688_ACC_ODR_896HZ = 0x03,      // 6DOF, 1000Hz accel-only
    W688_ACC_ODR_448HZ = 0x04,      // 6DOF, 500Hz accel-only
    W688_ACC_ODR_224HZ = 0x05,      // 6DOF, 250Hz accel-only
    W688_ACC_ODR_112HZ = 0x06,      // 6DOF, 125Hz accel-only
    W688_ACC_ODR_56HZ = 0x07,       // 6DOF, 62.5Hz accel-only
    W688_ACC_ODR_28HZ = 0x08,       // 6DOF, 31.25Hz accel-only
    W688_ACC_ODR_LOWPOWER_128HZ = 0x0C,
    W688_ACC_ODR_LOWPOWER_21HZ = 0x0D,
    W688_ACC_ODR_LOWPOWER_11HZ = 0x0E,
    W688_ACC_ODR_LOWPOWER_3HZ = 0x0F,
} w688AccOdr_e;

typedef enum {
    W688_GYR_RANGE_16DPS = 0x00 << 4,
    W688_GYR_RANGE_32DPS = 0x01 << 4,
    W688_GYR_RANGE_64DPS = 0x02 << 4,
    W688_GYR_RANGE_128DPS = 0x03 << 4,
    W688_GYR_RANGE_256DPS = 0x04 << 4,
    W688_GYR_RANGE_512DPS = 0x05 << 4,
    W688_GYR_RANGE_1024DPS = 0x06 << 4,
    W688_GYR_RANGE_2048DPS = 0x07 << 4,
} w688GyroRange_e;

typedef enum {
    W688_GYR_ODR_7174HZ = 0x00,
    W688_GYR_ODR_3587HZ = 0x01,
    W688_GYR_ODR_1793HZ = 0x02,
    W688_GYR_ODR_896HZ = 0x03,
    W688_GYR_ODR_448HZ = 0x04,
    W688_GYR_ODR_224HZ = 0x05,
    W688_GYR_ODR_112HZ = 0x06,
    W688_GYR_ODR_56HZ = 0x07,
    W688_GYR_ODR_28HZ = 0x08,
} w688GyroOdr_e;

// WHO_AM_I value
#define W688_WHO_AM_I_VALUE         0x05

// CTRL1
#define W688_CTRL1_ADDR_AI_EN       (1U << 6)
#define W688_CTRL1_BE               (1U << 5)

// CTRL5 filter configuration
#define W688_CTRL5_ACC_LPF_ENABLE   (1U << 0)
#define W688_CTRL5_ACC_LPF_MODE_0   (0x00U << 1)
#define W688_CTRL5_ACC_LPF_MODE_1   (0x01U << 1)
#define W688_CTRL5_ACC_LPF_MODE_2   (0x02U << 1)
#define W688_CTRL5_ACC_LPF_MODE_3   (0x03U << 1)
#define W688_CTRL5_GYR_LPF_ENABLE   (1U << 4)
#define W688_CTRL5_GYR_LPF_MODE_0   (0x00U << 5)
#define W688_CTRL5_GYR_LPF_MODE_1   (0x01U << 5)
#define W688_CTRL5_GYR_LPF_MODE_2   (0x02U << 5)
#define W688_CTRL5_GYR_LPF_MODE_3   (0x03U << 5)

// CTRL7 sensor enable / sync mode
#define W688_CTRL7_SYNCSAMPLE       (1U << 7)

// RESET command
#define W688_SOFT_RESET_CMD         0xB0
#define W688_CTRL9_CMD_ACK          0x00
#define W688_CTRL9_CMD_ON_DEMAND_CALI 0xA2
#define W688_COD_STATUS_OK          0x00

// 上电/复位延时配置
#define W688_POWERUP_DELAY_MSEC     20U
#define W688_RESET_DELAY_MSEC       20U
#define W688_HARDWARE_INIT_MAX_TRIES 5U
#define W688_CTRL9_ACK_DELAY_MSEC   10U
#define W688_ON_DEMAND_CALI_DELAY_MSEC 2000U

// SPI 读寄存器时，需要在寄存器地址最高位加上读标志（MSB=1）
#define W688_READ_FLAG              0x80

#define W688_ACC_AVAIL_FLAG          (1U << 1)
#define W688_GYRO_AVAIL_FLAG         (1U << 0)
#define W688_FRAME_SIZE              12U

// Current working configuration for Betaflight
#define W688_DEFAULT_CTRL1          (W688_CTRL1_ADDR_AI_EN | W688_CTRL1_BE)
#define W688_DEFAULT_ACC_RANGE      W688_ACC_RANGE_16G
#define W688_DEFAULT_ACC_ODR        W688_ACC_ODR_3587HZ
#define W688_DEFAULT_GYR_RANGE      W688_GYR_RANGE_2048DPS
#define W688_DEFAULT_GYR_ODR        W688_GYR_ODR_3587HZ

// Sensitivities for selected ranges
#define W688_ACCEL_SSVT_16G         2048.0f
#define W688_GYRO_SSVT_2048DPS      16.0f

typedef struct w688Cache_s {
    int16_t accRaw[XYZ_AXIS_COUNT];
    int16_t gyroRaw[XYZ_AXIS_COUNT];
    bool valid;
} w688Cache_t;

static w688Cache_t w688Cache;

// 辅助函数：读取一个或多个寄存器（自动添加 SPI 读标志位 0x80）
static bool w688_Read_Registers(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return spiReadRegBufRB(dev, reg | W688_READ_FLAG, buf, len);
}

static void w688_Write_Register_Blind(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    spiWriteReg(dev, reg, val);
    delayMicroseconds(50);
}

// 辅助函数：写入单个寄存器
static bool w688_Write_Register(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    for (int i = 0;i < 5; i++) { // 最多重试 5 次
        spiWriteReg(dev, reg, val); 
        uint8_t readback;
        if (w688_Read_Registers(dev, reg, &readback, 1) && readback == val) {
            return true; // 写入成功且读回值正确，完成写入
        }
    }
    return false; // 所有重试均失败
}

static bool w688ConfigAcc(extDevice_t *dev, w688AccRange_e range, w688AccOdr_e odr, bool lpfEnable)
{
    const uint8_t ctrl2 = (uint8_t)range | (uint8_t)odr;
    if (!w688_Write_Register(dev, W688_REG_CTRL2, ctrl2)) {
        return false;
    }

    uint8_t ctrl5 = 0;
    if (!w688_Read_Registers(dev, W688_REG_CTRL5, &ctrl5, 1)) {
        return false;
    }

    ctrl5 &= 0xF0;
    if (lpfEnable) {
        ctrl5 |= W688_CTRL5_ACC_LPF_ENABLE | W688_CTRL5_ACC_LPF_MODE_3;
    } else {
        ctrl5 &= ~W688_CTRL5_ACC_LPF_ENABLE;
    }

    return w688_Write_Register(dev, W688_REG_CTRL5, ctrl5);
}

static bool w688ConfigGyro(extDevice_t *dev, w688GyroRange_e range, w688GyroOdr_e odr, bool lpfEnable)
{
    const uint8_t ctrl3 = (uint8_t)range | (uint8_t)odr;
    if (!w688_Write_Register(dev, W688_REG_CTRL3, ctrl3)) {
        return false;
    }

    uint8_t ctrl5 = 0;
    if (!w688_Read_Registers(dev, W688_REG_CTRL5, &ctrl5, 1)) {
        return false;
    }

    ctrl5 &= 0x0F;
    if (lpfEnable) {
        ctrl5 |= W688_CTRL5_GYR_LPF_ENABLE | W688_CTRL5_GYR_LPF_MODE_3;
    } else {
        ctrl5 &= ~W688_CTRL5_GYR_LPF_ENABLE;
    }

    return w688_Write_Register(dev, W688_REG_CTRL5, ctrl5);
}

static bool w688EnableSensors(extDevice_t *dev, w688SensorEnable_e enableFlags)
{
    const uint8_t ctrl7 = (uint8_t)enableFlags;
    if (!w688_Write_Register(dev, W688_REG_CTRL7, ctrl7)) {
        return false;
    }

    delay(2);
    return true;
}

// 执行软复位，并等待芯片就绪
static bool w688HardwareInit(const extDevice_t *dev)
{
    delay(W688_POWERUP_DELAY_MSEC);
    // 向 RESET 寄存器写入复位命令
    for (uint8_t i = 0; i < W688_HARDWARE_INIT_MAX_TRIES; i++) {
        w688_Write_Register_Blind(dev, W688_REG_RESET, W688_SOFT_RESET_CMD);
        // 等待复位完成（典型值 <10ms，保守等待 20ms）
        delay(W688_RESET_DELAY_MSEC);

        // 轮询复位完成标志寄存器（0x4D），等待复位完成
        uint8_t reset_done = 0;
        for (uint16_t j = 0; j < 200; j++) { // 最多等待 100 次读取（总计约 1s）
            if (!w688_Read_Registers(dev, W688_REG_RESET_DONE, &reset_done, 1)) {
                continue; // 读取失败，重试
            }
            if (reset_done == 0x80) { // bit0=1 表示复位完成

                return true;
            }
            delay(1); // 等待 1ms 后重试
        }
        // 读取 WHO_AM_I 寄存器确认芯片 ID
        uint8_t chip_id = 0;
        if (w688_Read_Registers(dev, W688_REG_WHO_AM_I, &chip_id, 1) &&
            chip_id == W688_WHO_AM_I_VALUE) {
            return true;
        }
    }
    return false; // 复位失败
}

static bool w688OnDemandGyroGainCalibration(extDevice_t *dev)
{
    uint8_t codStatus = 0xFF;

    w688_Write_Register_Blind(dev, W688_REG_CTRL7, W688_DISABLE_ALL);
    delay(2);

    w688_Write_Register_Blind(dev, W688_REG_CTRL9, W688_CTRL9_CMD_ON_DEMAND_CALI);
    delay(W688_ON_DEMAND_CALI_DELAY_MSEC);

    w688_Write_Register_Blind(dev, W688_REG_CTRL9, W688_CTRL9_CMD_ACK);
    delay(W688_CTRL9_ACK_DELAY_MSEC);

    if (!w688_Read_Registers(dev, W688_REG_COD_STATUS, &codStatus, 1)) {
        return false;
    }

    return codStatus == W688_COD_STATUS_OK;
}

uint8_t w688Detect(const extDevice_t *dev)
{
    uint8_t id = 0;
    if (w688_Read_Registers(dev, W688_REG_WHO_AM_I, &id, 1) &&
    id == W688_WHO_AM_I_VALUE) {
        return W688_SPI; // 识别成功
    }

    return MPU_NONE;
}

// 配置 W688 寄存器
static bool w688Configure(extDevice_t *dev)
{
    // 先关闭传感器输出，避免配置过程中数据乱跳
    if (!w688_Write_Register(dev, W688_REG_CTRL7, W688_DISABLE_ALL)) {
        return false;
    }

    // CTRL1 对齐 EVB 参考驱动：自动地址递增 + 当前可工作的字节序配置。
    if (!w688_Write_Register(dev, W688_REG_CTRL1, W688_DEFAULT_CTRL1)) {
        return false;
    }

    if (!w688ConfigAcc(dev, W688_DEFAULT_ACC_RANGE, W688_DEFAULT_ACC_ODR, true)) {
        return false;
    }

    if (!w688ConfigGyro(dev, W688_DEFAULT_GYR_RANGE, W688_DEFAULT_GYR_ODR, true)) {
        return false;
    }

    if (!w688EnableSensors(dev, W688_ACCGYR_ENABLE)) {
        return false;
    }

    return true;
}

static void w688ParseFrame(const uint8_t *frame)
{
    w688Cache.accRaw[X] = (int16_t)((frame[1] << 8) | frame[0]);
    w688Cache.accRaw[Y] = (int16_t)((frame[3] << 8) | frame[2]);
    w688Cache.accRaw[Z] = (int16_t)((frame[5] << 8) | frame[4]);

    w688Cache.gyroRaw[X] = (int16_t)((frame[7] << 8) | frame[6]);
    w688Cache.gyroRaw[Y] = (int16_t)((frame[9] << 8) | frame[8]);
    w688Cache.gyroRaw[Z] = (int16_t)((frame[11] << 8) | frame[10]);
    w688Cache.valid = true;
}

// 陀螺仪初始化函数
void w688SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    memset(&w688Cache, 0, sizeof(w688Cache));

    if (!w688HardwareInit(dev)) {
        return;
    }

    (void)w688OnDemandGyroGainCalibration(dev);

    if (!w688Configure(dev)) {
        return;
    }

    gyro->scale = 1.0f / W688_GYRO_SSVT_2048DPS;
    spiSetClkDivisor(dev, spiCalculateDivider(W688_MAX_SPI_CLK_HZ));
}

// 陀螺仪初始化函数
void w688SpiAccInit(accDev_t *acc)
{
    // Configuration already done in gyro init
    acc->acc_1G = W688_ACCEL_SSVT_16G; // raw value per 1G
}

/**
 * @brief 陀螺仪读取函数 —— 仅读取陀螺仪数据（6字节）
 */
bool w688GyroRead(gyroDev_t *gyro)
{   

    extDevice_t *dev = &gyro->dev;
    uint8_t status0 = 0;
    uint8_t frame[W688_FRAME_SIZE];

    if (!w688_Read_Registers(dev, W688_REG_STATUS0, &status0, 1)) {
        return false;
    }

    if (!(status0 & W688_GYRO_AVAIL_FLAG)) {
        return false;
    }

    if (!w688_Read_Registers(dev, W688_REG_AX_L, frame, sizeof(frame))) {
        return false;
    }

    w688ParseFrame(frame);
    gyro->gyroADCRaw[X] = w688Cache.gyroRaw[X];
    gyro->gyroADCRaw[Y] = w688Cache.gyroRaw[Y];
    gyro->gyroADCRaw[Z] = w688Cache.gyroRaw[Z];
    
    return true;
}

/**
 * @brief 加速度计读取函数 —— 仅读取加速度计数据（6字节）
 */
bool w688AccRead(accDev_t *acc)
{

    if (!w688Cache.valid) {
        return false;
    }

    acc->ADCRaw[X] = w688Cache.accRaw[X];
    acc->ADCRaw[Y] = w688Cache.accRaw[Y];
    acc->ADCRaw[Z] = w688Cache.accRaw[Z];
    return true;

}

// 加速度计检测回调函数（由 accDetect 调用）
bool w688SpiAccDetect(accDev_t *acc)
{
    // 检查是否已通过 w688Detect 识别为此型号
    if (acc->mpuDetectionResult.sensor != W688_SPI) {
        return false;
    }
    // 注册初始化和读取函数
    acc->initFn = w688SpiAccInit;
    acc->readFn = w688AccRead;
    return true;
}

// 陀螺仪检测回调函数（由 gyroDetect 调用）
bool w688SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != W688_SPI) {
        return false;
    }
    gyro->initFn = w688SpiGyroInit;
    gyro->readFn = w688GyroRead;
    return true;
}

#endif // USE_ACCGYRO_W688
