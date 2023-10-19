## Sử dụng nhiều module Vl53L0X với STM32F1x

Trong code này sử dụng module Bluepill (STM32F103C8T6)

## Sử dụng

0. Khai báo sử dụng I2C trong project

1. Tạo biến khai báo cảm biến

```c++
struct VL53L0X myTOFsensor = {
        .hi2c = &hi2c1, // Kênh I2C sử dụng
        .io_2v8 = true,
        .address = 0x29, // Địa chỉ I2C, mặc định là 0x29
        .io_timeout = 500,
        .did_timeout = false
};
```

2. Kích hoạt trong USER CODE BEGIN 2

```c++
int main(void) {
    /* USER CODE BEGIN 2 */

    if( VL53L0X_init(&myTOFsensor) ){
        // success - do nothing
    }else{
        // error - Stop
        return 0;
    }

    // Khai báo sử dụng cảm biến ở LONG_RANGE và HIGH_SPEED
    VL53L0X_setSignalRateLimit(&myTOFsensor, 0.1);
	VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodPreRange, 18);
	VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodFinalRange, 14);
    VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20000);

    // Khai báo hoạt động ở chế độ đo liên tục
    VL53L0X_startContinuous(&myTOFsensor, 0);

    /* USER CODE END 2 */

    while (1) {
        /* USER CODE BEGIN 3 */

        // Đọc giá trị
        uint16_t value = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
        if ( VL53L0X_timeoutOccurred(&myTOFsensor) ) {
            // Đọc lỗi
        }
        
        HAL_Delay(500);

    }
    /* USER CODE END 3 */

```