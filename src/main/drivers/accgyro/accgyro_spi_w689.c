/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
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

#ifdef USE_ACCGYRO_W689

#include "common/axis.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_w689.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#define W689_MAX_SPI_CLK_HZ                 10000000

typedef enum {
    W689_REG_WHO_AM_I = 0x01,
    W689_REG_COM_CONF = 0x04,
    W689_REG_INT1_OUT_SEL1 = 0x05,
    W689_REG_INT1_OUT_SEL2 = 0x06,
    W689_REG_INT2_OUT_SEL1 = 0x07,
    W689_REG_INT2_OUT_SEL2 = 0x08,
    W689_REG_DATA_STAT = 0x0B,
    W689_REG_ACC_XH = 0x0C,
    W689_REG_FIFO_CFG0 = 0x1C,
    W689_REG_FIFO_CFG1 = 0x1D,
    W689_REG_AOI1_CFG = 0x30,
    W689_REG_AOI1_THS = 0x32,
    W689_REG_AOI1_DURATION = 0x33,
    W689_REG_ACC_CONF = 0x40,
    W689_REG_ACC_RANGE = 0x41,
    W689_REG_GYR_CONF = 0x42,
    W689_REG_GYR_RANGE = 0x43,
    W689_REG_SOFT_RST = 0x4A,
    W689_REG_PWR_CTRL = 0x7D,
    W689_REG_SEG_SEL = 0x7F,
    W689_REG_I2C_UN = 0x6F,
} w689Register_e;

typedef enum {
    W689_ACC_RANGE_2G = 0x00,
    W689_ACC_RANGE_4G = 0x01,
    W689_ACC_RANGE_8G = 0x02,
    W689_ACC_RANGE_16G = 0x03,
} w689AccRange_e;

typedef enum {
    W689_GYR_RANGE_2000DPS = 0x00,
    W689_GYR_RANGE_1000DPS = 0x01,
    W689_GYR_RANGE_500DPS = 0x02,
    W689_GYR_RANGE_250DPS = 0x03,
    W689_GYR_RANGE_125DPS = 0x04,
} w689GyroRange_e;

#define W689_WHO_AM_I_VALUE                 0x6A
#define W689_READ_FLAG                      0x80

#define W689_COM_CONF_BDU                   (1U << 6)
#define W689_COM_CONF_ADDR_AUTO             (1U << 4)
#define W689_COM_CONF_BOOT                  (1U << 7)

#define W689_DATA_STAT_GYR_CONF_ERR         (1U << 5)
#define W689_DATA_STAT_ACC_CONF_ERR         (1U << 4)
#define W689_DATA_STAT_GYR_DRDY             (1U << 1)

#define W689_PWR_CTRL_TEMP_ENABLE           (1U << 3)
#define W689_PWR_CTRL_ACC_ENABLE            (1U << 2)
#define W689_PWR_CTRL_GYR_ENABLE            (1U << 1)
#define W689_PWR_CTRL_DISABLE_ALL           0x00
#define W689_PWR_CTRL_ACCGYR_ENABLE         (W689_PWR_CTRL_TEMP_ENABLE | W689_PWR_CTRL_ACC_ENABLE | W689_PWR_CTRL_GYR_ENABLE)

#define W689_SEG_MAIN                       0x00
#define W689_SEG_I2C_SPI                    0x83
#define W689_SEG_DIG_CTRL                   0x8C
#define W689_I2C_DISABLE                    0xB4

#define W689_SOFT_RESET_CMD                 0xA5
#define W689_SPI_PREPARE_CMD                0x66
#define W689_SPI_PREPARE_CLEAR              0x00

#define W689_ACC_FILTER_PERF                (1U << 7)
#define W689_ACC_BWP_NORM_AVG4              (2U << 4)
#define W689_ACC_ODR_1600HZ                 0x0C
#define W689_GYR_FILTER_PERF                (1U << 7)
#define W689_GYR_BWP_OSR4_AVG1              (0U << 4)
#define W689_GYR_ODR_3200HZ                 0x0D

#define W689_DEFAULT_COM_CONF               (W689_COM_CONF_BDU | W689_COM_CONF_ADDR_AUTO)
#define W689_DEFAULT_ACC_CONF               (W689_ACC_FILTER_PERF | W689_ACC_BWP_NORM_AVG4 | W689_ACC_ODR_1600HZ)
#define W689_DEFAULT_GYR_CONF               (W689_GYR_FILTER_PERF | W689_GYR_BWP_OSR4_AVG1 | W689_GYR_ODR_3200HZ)
#define W689_DEFAULT_ACC_RANGE              W689_ACC_RANGE_16G
#define W689_DEFAULT_GYR_RANGE              W689_GYR_RANGE_2000DPS
#define W689_FIFO_DISABLED                  0x00
#define W689_FIFO_BYPASS                    0x00

#define W689_ACCEL_SSVT_16G                 2048
#define W689_FRAME_SIZE                     12U

#define W689_POWERUP_DELAY_MSEC             20U
#define W689_RESET_DELAY_MSEC               200U
#define W689_CONFIG_SETTLE_MSEC             2U
#define W689_ENABLE_SETTLE_MSEC             250U

typedef struct w689Cache_s {
    int16_t accRaw[XYZ_AXIS_COUNT];
    int16_t gyroRaw[XYZ_AXIS_COUNT];
    bool valid;
} w689Cache_t;

static w689Cache_t w689Cache;

static bool w689ReadRegisters(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return spiReadRegBufRB(dev, reg | W689_READ_FLAG, buf, len);
}

static void w689WriteRegisterBlind(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    spiWriteReg(dev, reg, val);
    delayMicroseconds(50);
}

static bool w689WriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    for (uint8_t i = 0; i < 5; i++) {
        w689WriteRegisterBlind(dev, reg, val);

        uint8_t readback = 0;
        if (w689ReadRegisters(dev, reg, &readback, 1) && readback == val) {
            return true;
        }
    }

    return false;
}

static void w689SelectSegment(const extDevice_t *dev, uint8_t segment)
{
    w689WriteRegisterBlind(dev, W689_REG_SEG_SEL, segment);
    delay(W689_CONFIG_SETTLE_MSEC);
}

static void w689PrepareSpiMode(const extDevice_t *dev)
{
    w689SelectSegment(dev, W689_SEG_MAIN);
    w689WriteRegisterBlind(dev, W689_REG_SOFT_RST, W689_SPI_PREPARE_CMD);
    delay(W689_CONFIG_SETTLE_MSEC);

    w689SelectSegment(dev, W689_SEG_I2C_SPI);
    w689WriteRegisterBlind(dev, W689_REG_I2C_UN, W689_I2C_DISABLE);
    w689WriteRegisterBlind(dev, W689_REG_I2C_UN, W689_I2C_DISABLE);
    delay(W689_CONFIG_SETTLE_MSEC);

    w689SelectSegment(dev, W689_SEG_MAIN);
    w689WriteRegisterBlind(dev, W689_REG_SOFT_RST, W689_SPI_PREPARE_CLEAR);
    delay(W689_CONFIG_SETTLE_MSEC);
}

static bool w689PowerDown(const extDevice_t *dev)
{
    w689SelectSegment(dev, W689_SEG_MAIN);

    if (!w689WriteRegister(dev, W689_REG_PWR_CTRL, W689_PWR_CTRL_DISABLE_ALL)) {
        return false;
    }

    delay(W689_CONFIG_SETTLE_MSEC);
    return true;
}

static bool w689SoftReset(const extDevice_t *dev)
{
    w689SelectSegment(dev, W689_SEG_MAIN);

    w689WriteRegisterBlind(dev, W689_REG_COM_CONF, W689_COM_CONF_BOOT);
    w689WriteRegisterBlind(dev, W689_REG_SOFT_RST, W689_SOFT_RESET_CMD);
    w689WriteRegisterBlind(dev, W689_REG_SOFT_RST, W689_SOFT_RESET_CMD);
    delay(W689_RESET_DELAY_MSEC);

    w689PrepareSpiMode(dev);

    uint8_t id = 0;
    return w689ReadRegisters(dev, W689_REG_WHO_AM_I, &id, 1) && id == W689_WHO_AM_I_VALUE;
}

static bool w689Configure(extDevice_t *dev)
{
    w689SelectSegment(dev, W689_SEG_MAIN);

    w689WriteRegisterBlind(dev, W689_REG_PWR_CTRL, W689_PWR_CTRL_ACCGYR_ENABLE);
    delay(5);
    w689WriteRegisterBlind(dev, W689_REG_PWR_CTRL, W689_PWR_CTRL_ACCGYR_ENABLE);
    delay(W689_ENABLE_SETTLE_MSEC);

    w689WriteRegisterBlind(dev, W689_REG_AOI1_CFG, 0x00);
    w689WriteRegisterBlind(dev, W689_REG_AOI1_THS, 0xFF);
    w689WriteRegisterBlind(dev, W689_REG_AOI1_DURATION, 0xFF);
    w689WriteRegisterBlind(dev, W689_REG_INT1_OUT_SEL1, 0x00);
    w689WriteRegisterBlind(dev, W689_REG_INT1_OUT_SEL2, 0x00);
    w689WriteRegisterBlind(dev, W689_REG_INT2_OUT_SEL1, 0x00);
    w689WriteRegisterBlind(dev, W689_REG_INT2_OUT_SEL2, 0x00);
    w689WriteRegisterBlind(dev, W689_REG_FIFO_CFG0, W689_FIFO_DISABLED);
    w689WriteRegisterBlind(dev, W689_REG_FIFO_CFG1, W689_FIFO_BYPASS);

    w689WriteRegisterBlind(dev, W689_REG_ACC_CONF, W689_DEFAULT_ACC_CONF);
    delay(W689_CONFIG_SETTLE_MSEC);

    w689WriteRegisterBlind(dev, W689_REG_ACC_RANGE, W689_DEFAULT_ACC_RANGE);

    w689WriteRegisterBlind(dev, W689_REG_GYR_CONF, W689_DEFAULT_GYR_CONF);
    delay(W689_CONFIG_SETTLE_MSEC);

    w689WriteRegisterBlind(dev, W689_REG_GYR_RANGE, W689_DEFAULT_GYR_RANGE);
    w689WriteRegisterBlind(dev, W689_REG_GYR_RANGE, W689_DEFAULT_GYR_RANGE);

    w689WriteRegisterBlind(dev, W689_REG_COM_CONF, W689_DEFAULT_COM_CONF);
    delay(W689_CONFIG_SETTLE_MSEC);

    w689SelectSegment(dev, W689_SEG_DIG_CTRL);
    uint8_t digCtrl = 0;
    if (w689ReadRegisters(dev, 0x30, &digCtrl, 1)) {
        digCtrl &= ~0x03;
        w689WriteRegisterBlind(dev, 0x30, digCtrl);
    }
    w689SelectSegment(dev, W689_SEG_MAIN);

    return true;
}

static bool w689HardwareInit(const extDevice_t *dev)
{
    delay(W689_POWERUP_DELAY_MSEC);

    w689PrepareSpiMode(dev);

    uint8_t id = 0;
    if (!w689ReadRegisters(dev, W689_REG_WHO_AM_I, &id, 1) || id != W689_WHO_AM_I_VALUE) {
        return false;
    }

    if (!w689PowerDown(dev)) {
        return false;
    }

    return w689SoftReset(dev);
}

uint8_t w689Detect(const extDevice_t *dev)
{
    w689PrepareSpiMode(dev);

    uint8_t id = 0;
    if (w689ReadRegisters(dev, W689_REG_WHO_AM_I, &id, 1) && id == W689_WHO_AM_I_VALUE) {
        return W689_SPI;
    }

    return MPU_NONE;
}

static void w689ParseFrame(const uint8_t *frame)
{
    w689Cache.accRaw[X] = (int16_t)((frame[0] << 8) | frame[1]);
    w689Cache.accRaw[Y] = (int16_t)((frame[2] << 8) | frame[3]);
    w689Cache.accRaw[Z] = (int16_t)((frame[4] << 8) | frame[5]);

    w689Cache.gyroRaw[X] = (int16_t)((frame[6] << 8) | frame[7]);
    w689Cache.gyroRaw[Y] = (int16_t)((frame[8] << 8) | frame[9]);
    w689Cache.gyroRaw[Z] = (int16_t)((frame[10] << 8) | frame[11]);

    w689Cache.valid = true;
}

void w689SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    memset(&w689Cache, 0, sizeof(w689Cache));

    gyro->scale = GYRO_SCALE_2000DPS;
    gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
    spiSetClkDivisor(dev, spiCalculateDivider(W689_MAX_SPI_CLK_HZ));

    if (!w689HardwareInit(dev)) {
        return;
    }

    if (!w689Configure(dev)) {
        return;
    }
}

void w689SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = W689_ACCEL_SSVT_16G;
}

bool w689GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    uint8_t status = 0;
    uint8_t frame[W689_FRAME_SIZE];

    if (!w689ReadRegisters(dev, W689_REG_DATA_STAT, &status, 1)) {
        return false;
    }

    if (status & (W689_DATA_STAT_GYR_CONF_ERR | W689_DATA_STAT_ACC_CONF_ERR)) {
        return false;
    }

    if (!(status & W689_DATA_STAT_GYR_DRDY)) {
        return false;
    }

    if (!w689ReadRegisters(dev, W689_REG_ACC_XH, frame, sizeof(frame))) {
        return false;
    }

    w689ParseFrame(frame);

    gyro->gyroADCRaw[X] = w689Cache.gyroRaw[X];
    gyro->gyroADCRaw[Y] = w689Cache.gyroRaw[Y];
    gyro->gyroADCRaw[Z] = w689Cache.gyroRaw[Z];

    return true;
}

bool w689AccRead(accDev_t *acc)
{
    if (!w689Cache.valid) {
        return false;
    }

    acc->ADCRaw[X] = w689Cache.accRaw[X];
    acc->ADCRaw[Y] = w689Cache.accRaw[Y];
    acc->ADCRaw[Z] = w689Cache.accRaw[Z];
    return true;
}

bool w689SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != W689_SPI) {
        return false;
    }

    acc->initFn = w689SpiAccInit;
    acc->readFn = w689AccRead;
    return true;
}

bool w689SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != W689_SPI) {
        return false;
    }

    gyro->initFn = w689SpiGyroInit;
    gyro->readFn = w689GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;
    gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
    return true;
}

#endif // USE_ACCGYRO_W689
