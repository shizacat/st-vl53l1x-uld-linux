/*
Гланвй файл для тестирования
*/

#include "core/VL53L1X_api.h"

void print_error(const char* msg){
    printf("[E] %s\n", msg);
    exit(1);
};

int main(int argc, char *argv[])
{
    // Bus number hardcode in platform.c

    VL53L1X_Version_t version;
    uint8_t ret, dataReady, rangeStatus;
    uint16_t distance;
    uint16_t dev = 0x29;  // Address on i2c bus
  
    VL53L1X_GetSWVersion(&version);
    printf(
        "Version driver: %i.%i B:%i R:%i\n",
        version.major, version.minor, version.build, version.revision
    );
    
    printf("Start meashurent\n");
    ret = 0;
    if (VL53L1X_BootState(dev, &ret) !=0 ){
        print_error("Boot state");
    }
    if (ret == 0) {
        printf("Not booted\n");
        return 0;
    }

    if (VL53L1X_SensorInit(dev) != 0){
        print_error("Sensor init");
    }

    // Ms
    if (VL53L1X_SetInterMeasurementInMs(dev, 100) !=0)
        print_error("Set inter measurement");

    // (1=short, 2=long(default)
    if (VL53L1X_SetDistanceMode(dev, 1) !=0 )
        print_error("Set distance mode");

    // mm
    if (VL53L1X_SetOffset(dev, 28) != 0)
        print_error("Set offset");

    /* enable the ranging*/
    if (VL53L1X_StartRanging(dev) != 0)
        print_error("Start ranging");
    
    /* ranging loop */
    while(1){
        while(dataReady==0){
            if (VL53L1X_CheckForDataReady(dev, &dataReady) != 0)
                print_error("Check data ready");
        }
        dataReady = 0;
        if (VL53L1X_GetRangeStatus(dev, &rangeStatus) != 0)
            print_error("Get range status");
        printf("Range status: %i\n", rangeStatus);

        // mm
        if (VL53L1X_GetDistance(dev, &distance) != 0)
            print_error("Get distance");

        printf("Distance (mm): %i\n", distance);

        if (VL53L1X_ClearInterrupt(dev) != 0)
            print_error("Clear interrupt");
    }

    return 0;
}