#pragma once

#include <stdint.h>

#include "drivers/accgyro/accgyro.h"

uint8_t w689Detect(const extDevice_t *dev);

bool w689SpiAccDetect(accDev_t *acc);
bool w689SpiGyroDetect(gyroDev_t *gyro);

void w689SpiAccInit(accDev_t *acc);
void w689SpiGyroInit(gyroDev_t *gyro);

bool w689AccRead(accDev_t *acc);
bool w689GyroRead(gyroDev_t *gyro);
