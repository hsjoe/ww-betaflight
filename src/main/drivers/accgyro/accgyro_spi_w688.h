#pragma once

#include <stdint.h>
#include "drivers/accgyro/accgyro.h"

uint8_t w688Detect(const extDevice_t *dev);

bool w688SpiAccDetect(accDev_t *acc);
bool w688SpiGyroDetect(gyroDev_t *gyro);

void w688SpiAccInit(accDev_t *acc);
void w688SpiGyroInit(gyroDev_t *gyro);

bool w688AccRead(accDev_t *acc);
bool w688GyroRead(gyroDev_t *gyro);
