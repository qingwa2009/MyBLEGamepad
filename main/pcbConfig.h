#ifndef _PCB_CONFIG_H_
#define _PCB_CONFIG_H_

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define PIN_ROW_0 GPIO_NUM_15
#define PIN_ROW_1 GPIO_NUM_16
#define PIN_ROW_2 GPIO_NUM_17
#define PIN_ROW_3 GPIO_NUM_18

#define PIN_COL_0 GPIO_NUM_11
#define PIN_COL_1 GPIO_NUM_12
#define PIN_COL_2 GPIO_NUM_13
#define PIN_COL_3 GPIO_NUM_14

#define PIN_BTN_SELECT GPIO_NUM_0
#define PIN_SWITCH_ON_OFF GPIO_NUM_45
#define PIN_BTN_XBOX GPIO_NUM_46

#define PIN_CHARGING_STATE GPIO_NUM_39
#define PIN_CHARGING_STDBY_STATE GPIO_NUM_40

#define PIN_I2C_VCC GPIO_NUM_3
#define PIN_I2C_SCL GPIO_NUM_41
#define PIN_I2C_SDA GPIO_NUM_42

#define PIN_LED_1 GPIO_NUM_21
#define PIN_LED_2 GPIO_NUM_47
#define PIN_LED_3 GPIO_NUM_48
#define PIN_LED_4 GPIO_NUM_38

#define PIN_MOTOR_L GPIO_NUM_9
#define PIN_MOTOR_R GPIO_NUM_10
#define LEDC_MOTOR_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_MOTOR_L_TIMER LEDC_TIMER_1
#define LEDC_MOTOR_R_TIMER LEDC_TIMER_2
#define LEDC_MOTOR_L_CHANNEL LEDC_CHANNEL_0
#define LEDC_MOTOR_R_CHANNEL LEDC_CHANNEL_1
#define LEDC_MOTOR_FRQ_DEFAULT 10 //越大振感越弱

#define PIN_ADC_LT ADC_CHANNEL_0         //GPIO_NUM_1
#define PIN_ADC_RT ADC_CHANNEL_1         //GPIO_NUM_2
#define PIN_ADC_X ADC_CHANNEL_3          //GPIO_NUM_4
#define PIN_ADC_Y ADC_CHANNEL_4          //GPIO_NUM_5
#define PIN_ADC_Z ADC_CHANNEL_6          //GPIO_NUM_7
#define PIN_ADC_RZ ADC_CHANNEL_5         //GPIO_NUM_6
#define PIN_ADC_BATT_LEVEL ADC_CHANNEL_7 //GPIO_NUM_8

#define BATT_VOLTAGE_MULTIPLY_FACTOR 2 //分压测量电压

#pragma pack(push, 1)
typedef struct
{
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t Up : 1;
    uint8_t Right : 1;
    uint8_t Down : 1;
    uint8_t Left : 1;
    uint8_t LS : 1;
    uint8_t RS : 1;
    uint8_t LB : 1;
    uint8_t RB : 1;
    uint8_t Start : 1;
    uint8_t ext1 : 1;
    uint8_t ext2 : 1;
    uint8_t ext3 : 1;
} KeyMatrix_t;

typedef enum
{
    GAMEPAD_TYPE_XBOX = 0,
    GAMEPAD_TYPE_SWITCH_PRO = 1,
    GAMEPAD_TYPE_DS = 3, //跳过2，2是电量低指示灯
} Enum_Gamepad_Type;

#pragma pack(pop)

#ifdef __cplusplus
extern "C"
{
#endif

    //0~100
    extern void setMotorLevel(uint16_t lMotor, uint16_t rMotor);
    extern void setMotorFreq(uint16_t lFreq, uint16_t rFreq);
    extern void getMotorLevel(uint16_t *lMotor, uint16_t *rMotor);
    extern void enableIMU();
    extern void disableIMU();
    /**
     * accel range
     * 0 = +/- 2g
     * 1 = +/- 4g
     * 2 = +/- 8g
     * 3 = +/- 16g
     * gyro range
     * 0 = +/- 250 degrees/sec
     * 1 = +/- 500 degrees/sec
     * 2 = +/- 1000 degrees/sec
     * 3 = +/- 2000 degrees/sec
     * bandwidth 0 1
     * samplerate 0 1
    */
    extern void setIMUSensitivity(uint8_t accelRange, uint8_t gyroRange, uint8_t accelBandwidth, uint8_t gyroSampleRate);
    extern void calibrateIMU();
    //return accel offset int16_t * 3 gyro offset int16_t * 3
    extern int16_t *getIMUCalibrateOffset();
    extern void getIMUMotionData(int16_t outData[6]);
    extern int getIMUEnabled();
    //3300~4200 mv
    extern int getBatteryVoltage();
    //0~100
    extern int getBatteryLevel();
    extern int isBatteryCharging();
    extern int isBatteryChargeFull();
    extern void setLEDScanning(uint8_t isScanning);

#ifdef __cplusplus
}
#endif

#endif