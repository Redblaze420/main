/*
 * This file is part of the Trezor project, https://trezor.io/
 *
 * Copyright (c) SatoshiLabs
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MPU_H__
#define __MPU_H__

#ifdef KERNEL_MODE

void mpu_config_off(void);
void mpu_config_boardloader(void);
void mpu_config_bootloader(void);
void mpu_config_firmware_initial(void);
void mpu_config_firmware(void);
void mpu_config_prodtest_initial(void);
void mpu_config_prodtest(void);

typedef enum {
  MPU_MODE_DISABLED,
  MPU_MODE_DEFAULT,
  MPU_MODE_ASSETS,
  MPU_MODE_STORAGE,
  MPU_MODE_OTP,
  MPU_MODE_BOOTLOADER,
  MPU_MODE_BOARDLOADER,
  MPU_MODE_APP,
  MPU_MODE_SAES,
} mpu_mode_t;

void mpu_init(void);

mpu_mode_t mpu_reconfig(mpu_mode_t mode);

void mpu_restore(mpu_mode_t mode);

#endif  // KERNEL_MODE

#endif
