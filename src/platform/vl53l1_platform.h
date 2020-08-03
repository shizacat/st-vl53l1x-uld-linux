/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */
 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
// #include <memory.h>
#include <unistd.h>

#define I2C_M_RD 0x0001
#define I2C_M_WR 0x0000

typedef struct {
	uint32_t dummy;
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t     *pdata);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);

#ifdef __cplusplus
}
#endif

#endif
