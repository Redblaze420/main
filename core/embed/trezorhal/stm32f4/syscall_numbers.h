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

#ifndef SYSCALL_NUMBERS_H
#define SYSCALL_NUMBERS_H

// Syscalls numbers
#define SYSCALL_SYSTICK_CYCLES 1
#define SYSCALL_SYSTICK_MS 2
#define SYSCALL_SYSTICK_US 3
#define SYSCALL_SYSTICK_US_TO_CYCLES 4

#define SYSCALL_SECURE_SHUTDOWN 5
#define SYSCALL_REBOOT 6
#define SYSCALL_REBOOT_TO_BOOTLOADER 7

#define SYSCALL_WAIT_RANDOM 8
#define SYSCALL_RDI_REFRESH_SESSION_DELAY 9

#define SYSCALL_SHA256_INIT 10
#define SYSCALL_SHA256_UPDATE 11
#define SYSCALL_SHA256_FINAL 12
#define SYSCALL_SHA256_CALC 13

#define SYSCALL_DISPLAY_SET_BACKLIGHT 14
#define SYSCALL_DISPLAY_GET_BACKLIGHT 15
#define SYSCALL_DISPLAY_SET_ORIENTATION 16
#define SYSCALL_DISPLAY_GET_ORIENTATION 17
#define SYSCALL_DISPLAY_GET_FB_INFO 18
#define SYSCALL_DISPLAY_WAIT_FOR_SYNC 19
#define SYSCALL_DISPLAY_REFRESH 20

#define SYSCALL_USB_INIT 21
#define SYSCALL_USB_DEINIT 22
#define SYSCALL_USB_START 23
#define SYSCALL_USB_STOP 24
#define SYSCALL_USB_CONFIGURED 25

#define SYSCALL_USB_HID_ADD 26
#define SYSCALL_USB_HID_CAN_READ 27
#define SYSCALL_USB_HID_CAN_WRITE 28
#define SYSCALL_USB_HID_READ 29
#define SYSCALL_USB_HID_WRITE 30
#define SYSCALL_USB_HID_READ_SELECT 31
#define SYSCALL_USB_HID_READ_BLOCKING 32
#define SYSCALL_USB_HID_WRITE_BLOCKING 33
#define SYSCALL_USB_VCP_ADD 34
#define SYSCALL_USB_VCP_CAN_READ 35
#define SYSCALL_USB_VCP_CAN_WRITE 36
#define SYSCALL_USB_VCP_READ 37
#define SYSCALL_USB_VCP_WRITE 38
#define SYSCALL_USB_VCP_READ_BLOCKING 39
#define SYSCALL_USB_VCP_WRITE_BLOCKING 40
#define SYSCALL_USB_WEBUSB_ADD 41
#define SYSCALL_USB_WEBUSB_CAN_READ 42
#define SYSCALL_USB_WEBUSB_CAN_WRITE 43
#define SYSCALL_USB_WEBUSB_READ 44
#define SYSCALL_USB_WEBUSB_WRITE 45
#define SYSCALL_USB_WEBUSB_READ_SELECT 46
#define SYSCALL_USB_WEBUSB_READ_BLOCKING 47
#define SYSCALL_USB_WEBUSB_WRITE_BLOCKING 48

#define SYSCALL_SDCARD_POWER_ON 49
#define SYSCALL_SDCARD_POWER_OFF 50
#define SYSCALL_SDCARD_IS_PRESENT 51
#define SYSCALL_SDCARD_GET_CAPACITY 52
#define SYSCALL_SDCARD_READ_BLOCKS 53
#define SYSCALL_SDCARD_WRITE_BLOCKS 54

#define SYSCALL_UNIT_VARIANT_PRESENT 55
#define SYSCALL_UNIT_VARIANT_GET_COLOR 56
#define SYSCALL_UNIT_VARIANT_GET_PACKAGING 57
#define SYSCALL_UNIT_VARIANT_GET_BTCONLY 58
#define SYSCALL_UNIT_VARIANT_IS_SD_HOTSWAP_ENABLED 59

#define SYSCALL_SECRET_BOOTLOADER_LOCKED 60

#define SYSCALL_BUTTON_READ 61
#define SYSCALL_BUTTON_STATE_LEFT 62
#define SYSCALL_BUTTON_STATE_RIGHT 63

#define SYSCALL_TOUCH_GET_EVENT 64

#define SYSCALL_HAPTIC_SET_ENABLED 65
#define SYSCALL_HAPTIC_GET_ENABLED 66
#define SYSCALL_HAPTIC_TEST 67
#define SYSCALL_HAPTIC_PLAY 68
#define SYSCALL_HAPTIC_PLAY_CUSTOM 69

#define SYSCALL_OPTIGA_SIGN 70
#define SYSCALL_OPTIGA_CERT_SIZE 71
#define SYSCALL_OPTIGA_READ_CERT 72
#define SYSCALL_OPTIGA_READ_SEC 73
#define SYSCALL_OPTIGA_RANDOM_BUFFER 74

#define SYSCALL_STORAGE_INIT 75
#define SYSCALL_STORAGE_WIPE 76
#define SYSCALL_STORAGE_IS_UNLOCKED 77
#define SYSCALL_STORAGE_LOCK 78
#define SYSCALL_STORAGE_UNLOCK 79
#define SYSCALL_STORAGE_HAS_PIN 80
#define SYSCALL_STORAGE_PIN_FAILS_INCREASE 81
#define SYSCALL_STORAGE_GET_PIN_REM 82
#define SYSCALL_STORAGE_CHANGE_PIN 83
#define SYSCALL_STORAGE_ENSURE_NOT_WIPE_CODE 84
#define SYSCALL_STORAGE_HAS_WIPE_CODE 85
#define SYSCALL_STORAGE_CHANGE_WIPE_CODE 86
#define SYSCALL_STORAGE_HAS 87
#define SYSCALL_STORAGE_GET 88
#define SYSCALL_STORAGE_SET 89
#define SYSCALL_STORAGE_DELETE 90
#define SYSCALL_STORAGE_SET_COUNTER 91
#define SYSCALL_STORAGE_NEXT_COUNTER 92

#define SYSCALL_FLASH_AREA_GET_SIZE 93
#define SYSCALL_FLASH_AREA_GET_ADDRESS 94

#define SYSCALL_ENTROPY_GET 95

#define SYSCALL_TRANSLATIONS_WRITE 96
#define SYSCALL_TRANSLATIONS_READ 97
#define SYSCALL_TRANSLATIONS_ERASE 98
#define SYSCALL_TRANSLATIONS_AREA_BYTESIZE 99

#define SYSCALL_RNG_GET 100

// application entry point addresses
#define ENTRYPOINT_STACK (COREAPP_START + 0x00)
#define ENTRYPOINT_RESET (COREAPP_START + 0x04)
#define ENTRYPOINT_SAES_INVOKE (COREAPP_START + 0x08)
#define ENTRYPOINT_SAES_INPUT (COREAPP_START + 0x0C)
#define ENTRYPOINT_SAES_OUTPUT (COREAPP_START + 0x10)

#endif  // SYSCALL_NUMBERS_H