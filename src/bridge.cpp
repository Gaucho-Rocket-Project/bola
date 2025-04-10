#include "WiringPiTest.h"
#include "icm20948_api.h"

icm20948_read_fptr_t read_func = digitalRead;
icm20948_write_fptr_t write_func = digitalWrite;
icm20948_delay_us_fptr_t delay_func = usleep;