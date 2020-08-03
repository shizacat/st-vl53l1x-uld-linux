
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#define BUS_FILE_NAME "/dev/i2c-1"
int fd_bus_i2c = 0;

int get_fd_bus(){
	if (fd_bus_i2c == 0){
		fd_bus_i2c = open(BUS_FILE_NAME, O_RDWR);
		if (fd_bus_i2c < 0){
			fd_bus_i2c = 0;
			printf("Error open\n");
        	exit(1);
		}
	}
	return fd_bus_i2c;
}


int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t* data;
    uint8_t ret = 0;
    
	data = new uint8_t [2 + count];

    data[0] = (index >> 8) & 0xff;
    data[1] = index & 0xff;
    memcpy(&data[2], pdata, count);

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 2 + count;
    msgs[0].buf = data;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        ret = -1;
    delete data;
    return ret;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t data[2];

    data[0] = (index >> 8) & 0xff;
    data[1] = index & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 2;
    msgs[0].buf = data;

    msgs[1].addr = dev;
    msgs[1].flags = I2C_M_RD; // | I2C_M_NOSTART;
    msgs[1].len = count;
    msgs[1].buf = pdata;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t local_data[3];

    local_data[0] = (index >> 8) & 0xff;
    local_data[1] = index & 0xff;
    local_data[2] = data;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 3;
    msgs[0].buf = local_data;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t local_data[4];

    local_data[0] = (index >> 8) & 0xff;
    local_data[1] = index & 0xff;
    local_data[2] = (data >> 8) & 0xff;
	local_data[3] = data & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 4;
    msgs[0].buf = local_data;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t local_data[6];

    local_data[0] = (index >> 8) & 0xff;
    local_data[1] = index & 0xff;
    local_data[2] = (data >> 24) & 0xff;
	local_data[3] = (data >> 16) & 0xff;
	local_data[4] = (data >> 8) & 0xff;
	local_data[5] = data & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 6;
    msgs[0].buf = local_data;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t data_local[2];

    data_local[0] = (index >> 8) & 0xff;
    data_local[1] = index & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 2;
    msgs[0].buf = data_local;

    msgs[1].addr = dev;
    msgs[1].flags = I2C_M_RD; // | I2C_M_NOSTART;
    msgs[1].len = 1;
    msgs[1].buf = data;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t data_local[2], data_out[2];

    data_local[0] = (index >> 8) & 0xff;
    data_local[1] = index & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 2;
    msgs[0].buf = data_local;

    msgs[1].addr = dev;
    msgs[1].flags = I2C_M_RD; // | I2C_M_NOSTART;
    msgs[1].len = 2;
    msgs[1].buf = data_out;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;

    *data = ((uint16_t)data_out[0]<<8) + (uint16_t)data_out[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t data_local[2], data_out[4];

    data_local[0] = (index >> 8) & 0xff;
    data_local[1] = index & 0xff;

    msgs[0].addr = dev;
    msgs[0].flags = I2C_M_WR;
    msgs[0].len = 2;
    msgs[0].buf = data_local;

    msgs[1].addr = dev;
    msgs[1].flags = I2C_M_RD; // | I2C_M_NOSTART;
    msgs[1].len = 4;
    msgs[1].buf = data_out;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(get_fd_bus(), I2C_RDWR, &msgset) < 0)
        return -1;
    *data = ((uint32_t)data_out[0]<<24) +
        ((uint32_t)data_out[1]<<16) +
        ((uint32_t)data_out[2]<<8) +
        (uint32_t)data_out[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
    usleep(wait_ms * 1000);
	return 0;
}
