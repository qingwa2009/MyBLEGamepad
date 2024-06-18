#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/dedic_gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "pcbConfig.h"
#include "MyMeanFilter.h"
#include "MyKeypadMatrix.h"
#include "driver/ledc.h"
#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "ble_uuid.h"
#include "driver/i2c.h"
#include "MPU6050.h"
#include "helper.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "esp_task_wdt.h"
#include "esp_mac.h"

// #include "MPU6050_6Axis_MotionApps612.h"

#include "XboxOneGamepad.h"
#include "SwitchProController.h"
#include "DualSenseController.h"

#define ENABLE_LIGHT_SLEEP true

#define TIME_HOLDE_POW_OFF_MS 3000
#define BTN_PIN_LEVEL_STABLE_DELAY_NOP_COUNT 1 //按键电平切换稳定所需的空语句计数

#define DEFAULT_ADC_UNIT_ID ADC_UNIT_1
#define DEFAULT_ADC_ATTEN_DB ADC_ATTEN_DB_11
#define DEFAULT_ADC_BITWIDTH ADC_BITWIDTH_12

#define DEFAULT_SCAN_TIME_OUT_MS 30000 //扫描30s未连接自动关机

#define DELAY(x) vTaskDelay((x) / portTICK_PERIOD_MS)

static const char *TAG = "main";

static int isPowOnByPushBtn = 0;
static int _isPowOff = 0;
static volatile Enum_Gamepad_Type _gamepadType = GAMEPAD_TYPE_XBOX;
static volatile int _isScanning = 1;
static IMyGamepad *pGamepad = nullptr;

static const char *NVS_XBOX_BOND = "xbox_bond";
static const char *NVS_PRO_BOND = "pro_bond";
static const char *NVS_DS_BOND = "ds_bond";
static const char *mynvs = "mynvs";
static const char *mynvsKeyGPT = "gpt";
static const char *calinvs = "cali";
static const char *name_xbox = "MyXboxGamepad";
static const char *name_pro = "MyProGamepad";
static const char *name_ds = "MyDSGamepad";

static const char *cmdCaliMPU = "/m";
static const char *cmdCaliSticks = "/c";
static const char *cmdSaveSticks = "/s";
static const char *cmdSaveDeadZone = "/d";

static int16_t min_x, mid_x, max_x, min_y, mid_y, max_y, min_z, mid_z, max_z, min_rz, mid_rz, max_rz;
static int16_t dead_x, dead_y, dead_z, dead_rz;
//直接改存放绑定信息下nvs名称空间，实现多重绑定。
#include "../components/bt/nimble_nvs_namespace_variable.h"
const char *NIMBLE_NVS_NAMESPACE;

#ifdef __cplusplus
extern "C"
{
#endif
    void app_main(void);
#ifdef __cplusplus
}
#endif

/**
 * 接通电池。
*/
void connectBattery()
{
    //在bootloader已经做了输入输出的设置
    gpio_set_level(PIN_SWITCH_ON_OFF, 1);
}
void disconnectBattery()
{
    gpio_set_level(PIN_SWITCH_ON_OFF, 0);
}

int isBatteryConnected()
{
    //在bootloader已经做了输入输出的设置
    return gpio_get_level(PIN_SWITCH_ON_OFF);
}

//0b0000~0b1111
void setLEDState(uint8_t state)
{

    gpio_set_level(PIN_LED_1, state & 0b0001);
    gpio_set_level(PIN_LED_2, state & 0b0010);
    gpio_set_level(PIN_LED_3, state & 0b0100);
    gpio_set_level(PIN_LED_4, state & 0b1000);
}

void reset(bool byTWDT)
{

    //取消掉IO的保持功能，不然软重置也重置不了
    // gpio_hold_dis(PIN_LED_1);
    // gpio_hold_dis(PIN_LED_2);
    // gpio_hold_dis(PIN_LED_3);
    // gpio_hold_dis(PIN_LED_4);
    // gpio_hold_dis(PIN_I2C_VCC);
    // gpio_hold_dis(PIN_SWITCH_ON_OFF);
    if (byTWDT)
        esp_system_abort("reset by watch dog!!!");
    else
        esp_system_abort("reset by press xbox btn!!!");
}

void powOn()
{
    _isPowOff = 0;
    connectBattery();
}
/**
 * 关机
*/
void powOff()
{
    _isPowOff = 1;
    pGamepad->stop();
    setLEDState(0b0000);
    esp_wifi_bt_power_domain_off();
    disconnectBattery();
}

//xbox 按键还要负责开关机，所以按下时不返回按下，松开时才返回按下，下一帧返回松开
int checkXboxBtn(uint32_t dt)
{
    static int preXboxBtnState = 0;
    static uint32_t t = 0;
    static int enableXboxBtnAsPowBtn = 0; //按住开机时，关闭xbox按键的关机功能

    int isXboxBtnDown = !gpio_get_level(PIN_BTN_XBOX);
    if (!_isPowOff)
    {
        if (isXboxBtnDown)
        {
            if (enableXboxBtnAsPowBtn)
                t += dt;
            //长按超时
            if ((t) > (TIME_HOLDE_POW_OFF_MS))
            {
                powOff();
                return 0;
            }
            preXboxBtnState = 1;
            return 0;
        }
        else
        {
            enableXboxBtnAsPowBtn = 1;
            t = 0;
            if (preXboxBtnState > 0)
            {
                preXboxBtnState = 0;
                return 1;
            }
            else
            {
                preXboxBtnState = 0;
                return 0;
            }
        }
    }
    else
    {
        //长按关机，松手后。
        if (!isXboxBtnDown)
        {
            //重启
            reset(false);
        }
        preXboxBtnState = 0;
        return 0;
    }
}

adc_oneshot_unit_handle_t adcHandle = NULL;
adc_cali_handle_t adcCaliHandle = NULL;
void _initADCHandle()
{
    if (adcHandle != NULL)
        return;
    adc_oneshot_unit_init_cfg_t initCfg = {
        .unit_id = DEFAULT_ADC_UNIT_ID,
        .clk_src = (adc_oneshot_clk_src_t)0,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&initCfg, &adcHandle));
}

void initADCSticks()
{
    _initADCHandle();
    adc_oneshot_chan_cfg_t chanCfg = {
        .atten = DEFAULT_ADC_ATTEN_DB,
        .bitwidth = DEFAULT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_X, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_Y, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_Z, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_RZ, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_LT, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_RT, &chanCfg));
}

int getADCValue(adc_channel_t channel)
{
    int v = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adcHandle, channel, &v));
    return v;
}

int getADCValueAfterCali(adc_channel_t channel)
{
    int vRaw, vOut;
    ESP_ERROR_CHECK(adc_oneshot_read(adcHandle, channel, &vRaw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adcCaliHandle, vRaw, &vOut));
    // ESP_LOGI(TAG, "raw = %d v= %dmv \r\n", vRaw, vOut);
    return vOut;
}

void initMotor()
{
    gpio_reset_pin(PIN_MOTOR_L);
    gpio_reset_pin(PIN_MOTOR_R);

    ledc_timer_config_t ledcTCfg = {
        .speed_mode = LEDC_MOTOR_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_MOTOR_L_TIMER,
        .freq_hz = LEDC_MOTOR_FRQ_DEFAULT,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledcTCfg));

    ledc_channel_config_t ledcCCfg = {
        .gpio_num = PIN_MOTOR_L,
        .speed_mode = LEDC_MOTOR_SPEED_MODE,
        .channel = LEDC_MOTOR_L_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_MOTOR_L_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0},
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledcCCfg));

    ledcTCfg.timer_num = LEDC_MOTOR_R_TIMER;
    ESP_ERROR_CHECK(ledc_timer_config(&ledcTCfg));
    ledcCCfg.gpio_num = PIN_MOTOR_R;
    ledcCCfg.channel = LEDC_MOTOR_R_CHANNEL;
    ledcCCfg.timer_sel = LEDC_MOTOR_R_TIMER;
    ESP_ERROR_CHECK(ledc_channel_config(&ledcCCfg));
}

void setMotorFreq(uint16_t lFreq, uint16_t rFreq)
{
    ledc_set_freq(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_L_TIMER, lFreq);
    ledc_set_freq(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_R_TIMER, rFreq);
}
//0~100
void setMotorLevel(uint16_t lMotor, uint16_t rMotor)
{
    if (lMotor > 100)
        lMotor = 100;
    if (rMotor > 100)
        rMotor = 100;
    ledc_set_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_L_CHANNEL, lMotor * 40);
    ledc_update_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_L_CHANNEL);
    ledc_set_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_R_CHANNEL, rMotor * 40);
    ledc_update_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_R_CHANNEL);

    // ESP_LOGI(TAG, "LMotor: %d, RMotr: %d", lMotor, rMotor);
}
void getMotorLevel(uint16_t *lMotor, uint16_t *rMotor)
{
    *lMotor = ledc_get_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_L_CHANNEL) / 40;
    *rMotor = ledc_get_duty(LEDC_MOTOR_SPEED_MODE, LEDC_MOTOR_R_CHANNEL) / 40;
}

void initGamepadBLE(bool clearBond, bool powBoostMode)
{
    //改BLE地址，以区分不同手柄（注意这里的地址是最低字节在最高地址）
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Base BLE Public Addr: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    mac[1] += _gamepadType;
    esp_iface_mac_addr_set(mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Set BLE Public Addr: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    switch (_gamepadType)
    {
    case GAMEPAD_TYPE_SWITCH_PRO:
        //直接改存放绑定信息下nvs名称空间，实现多重绑定。
        NIMBLE_NVS_NAMESPACE = NVS_PRO_BOND;
        pGamepad = new SwitchProController(name_pro);
        break;
    case GAMEPAD_TYPE_DS:
        //直接改存放绑定信息下nvs名称空间，实现多重绑定。
        NIMBLE_NVS_NAMESPACE = NVS_DS_BOND;
        pGamepad = new DualSenseController(name_ds);
        break;
    default:
        //直接改存放绑定信息下nvs名称空间，实现多重绑定。
        NIMBLE_NVS_NAMESPACE = NVS_XBOX_BOND;
        pGamepad = new XboxOneGamepad(name_xbox);
        break;
    }
    if (clearBond)
    {
        pGamepad->clearBond();
    }
    pGamepad->setDeadZone(dead_x, dead_y, dead_z, dead_rz);
    pGamepad->start(powBoostMode);
}

void normalizeJoyStrickValue(int *_x, int *_y, int *_z, int *_rz)
{
    int x = *_x;
    int y = *_y;
    int z = *_z;
    int rz = *_rz;
    x = 4095 - x;
    z = 4095 - z;
    rz = 4095 - rz;
    x = (int)((x * 100L) / 4095);
    y = (int)((y * 100L) / 4095);
    z = (int)((z * 100L) / 4095);
    rz = (int)((rz * 100L) / 4095);

    // ESP_LOGI(TAG, ",x=%d,y=%d,z=%d,rz=%d", x, y, z, rz);

    x = (x == mid_x) ? 50 : ((x < mid_x) ? map(x, min_x, mid_x, 0, 50) : map(x, mid_x, max_x, 50, 100));
    y = (y == mid_y) ? 50 : ((y < mid_y) ? map(y, min_y, mid_y, 0, 50) : map(y, mid_y, max_y, 50, 100));
    z = (z == mid_z) ? 50 : ((z < mid_z) ? map(z, min_z, mid_z, 0, 50) : map(z, mid_z, max_z, 50, 100));
    rz = (rz == mid_rz) ? 50 : ((rz < mid_rz) ? map(rz, min_rz, mid_rz, 0, 50) : map(rz, mid_rz, max_rz, 50, 100));

    if (x < 0)
        x = 0;
    else if (x > 100)
        x = 100;
    if (y < 0)
        y = 0;
    else if (y > 100)
        y = 100;
    if (z < 0)
        z = 0;
    else if (z > 100)
        z = 100;
    if (rz < 0)
        rz = 0;
    else if (rz > 100)
        rz = 100;

    *_x = x;
    *_y = y;
    *_z = z;
    *_rz = rz;
}

void normalizeTriggerValue(int *_lt, int *_rt)
{
    int lt = *_lt;
    int rt = *_rt;
    lt /= 40;
    rt /= 40;
    if (lt > 100)
        lt = 100;
    else if (lt < 0)
        lt = 0;
    if (rt > 100)
        rt = 100;
    else if (rt < 0)
        rt = 0;
    *_lt = lt;
    *_rt = rt;
}

MPU6050 mpu;
static int isIMUEnabled = false;
void initI2C()
{
    i2c_config_t cfg;
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = PIN_I2C_SDA;
    cfg.scl_io_num = PIN_I2C_SCL;
    cfg.sda_pullup_en = 0;
    cfg.scl_pullup_en = 0;
    cfg.master.clk_speed = 400000;
    cfg.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_NUM_0, &cfg);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0));
}
void saveDeadZone(int16_t deadx, int16_t deady, int16_t deadz, int16_t deadrz)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_set_i16(h, "deadx", deadx);
    nvs_set_i16(h, "deady", deady);
    nvs_set_i16(h, "deadz", deadz);
    nvs_set_i16(h, "deadrz", deadrz);
    nvs_commit(h);
    nvs_close(h);
}
void loadDeadZone(int16_t *deadx, int16_t *deady, int16_t *deadz, int16_t *deadrz)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_get_i16(h, "deadx", deadx);
    nvs_get_i16(h, "deady", deady);
    nvs_get_i16(h, "deadz", deadz);
    nvs_get_i16(h, "deadrz", deadrz);
    nvs_close(h);
}
void saveSticksOffset(int16_t minx, int16_t midx, int16_t maxx, int16_t miny, int16_t midy, int16_t maxy, int16_t minz, int16_t midz, int16_t maxz, int16_t minrz, int16_t midrz, int16_t maxrz)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_set_i16(h, "minx", minx);
    nvs_set_i16(h, "midx", midx);
    nvs_set_i16(h, "maxx", maxx);
    nvs_set_i16(h, "miny", miny);
    nvs_set_i16(h, "midy", midy);
    nvs_set_i16(h, "maxy", maxy);
    nvs_set_i16(h, "minz", minz);
    nvs_set_i16(h, "midz", midz);
    nvs_set_i16(h, "maxz", maxz);
    nvs_set_i16(h, "minrz", minrz);
    nvs_set_i16(h, "midrz", midrz);
    nvs_set_i16(h, "maxrz", maxrz);
    nvs_commit(h);
    nvs_close(h);
}
void loadSticksOffset(int16_t *minx, int16_t *midx, int16_t *maxx, int16_t *miny, int16_t *midy, int16_t *maxy, int16_t *minz, int16_t *midz, int16_t *maxz, int16_t *minrz, int16_t *midrz, int16_t *maxrz)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_get_i16(h, "minx", minx);
    nvs_get_i16(h, "midx", midx);
    nvs_get_i16(h, "maxx", maxx);
    nvs_get_i16(h, "miny", miny);
    nvs_get_i16(h, "midy", midy);
    nvs_get_i16(h, "maxy", maxy);
    nvs_get_i16(h, "minz", minz);
    nvs_get_i16(h, "midz", midz);
    nvs_get_i16(h, "maxz", maxz);
    nvs_get_i16(h, "minrz", minrz);
    nvs_get_i16(h, "midrz", midrz);
    nvs_get_i16(h, "maxrz", maxrz);
    nvs_close(h);
}
void saveIMUCaliOffset(int16_t *offsets)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_set_i16(h, "acclx", offsets[0]);
    nvs_set_i16(h, "accly", offsets[1]);
    nvs_set_i16(h, "acclz", offsets[2]);
    nvs_set_i16(h, "gyrox", offsets[3]);
    nvs_set_i16(h, "gyroy", offsets[4]);
    nvs_set_i16(h, "gyroz", offsets[5]);
    nvs_commit(h);
    nvs_close(h);
}
void loadIMUCaliOffset(int16_t *acclx, int16_t *accly, int16_t *acclz, int16_t *gyrox, int16_t *gyroy, int16_t *gyroz)
{
    nvs_handle_t h;
    nvs_open(calinvs, NVS_READWRITE, &h);
    nvs_get_i16(h, "acclx", acclx);
    nvs_get_i16(h, "accly", accly);
    nvs_get_i16(h, "acclz", acclz);
    nvs_get_i16(h, "gyrox", gyrox);
    nvs_get_i16(h, "gyroy", gyroy);
    nvs_get_i16(h, "gyroz", gyroz);
    nvs_close(h);
}
int getIMUEnabled()
{
    return isIMUEnabled;
}
void disableIMU()
{
    gpio_set_level(PIN_I2C_VCC, 0);
    isIMUEnabled = 0;
}
void enableIMU()
{
    if (isIMUEnabled)
        return;
    isIMUEnabled = 1;

    gpio_reset_pin(PIN_I2C_VCC);
    gpio_set_direction(PIN_I2C_VCC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_I2C_VCC, 1);
    DELAY(10); //等上电稳定

    mpu.initialize();

    // Get Device ID
    uint8_t buffer[1];
    I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
    ESP_LOGI(TAG, "MPU6050 getDeviceID=0x%x", buffer[0]);

    int16_t acclx = 0, accly = 0, acclz = 0;
    int16_t gyrox = 0, gyroy = 0, gyroz = 0;
    loadIMUCaliOffset(&acclx, &accly, &acclz, &gyrox, &gyroy, &gyroz);

    ESP_LOGI(TAG, "Load IMU offset accl x y z: %6d %6d %6d", acclx, accly, acclz);
    ESP_LOGI(TAG, "Load IMU offset gyro x y z: %6d %6d %6d", gyrox, gyroy, gyroz);

    // // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(acclx);
    mpu.setYAccelOffset(accly);
    mpu.setZAccelOffset(acclz);
    mpu.setXGyroOffset(gyrox);
    mpu.setYGyroOffset(gyroy);
    mpu.setZGyroOffset(gyroz);

    uint8_t gyroRate = mpu.getDLPFMode() == 0 ? 8 : 1; //k
    uint8_t sampleDiv = mpu.getRate();

    ESP_LOGI(TAG, "MPU6050 Sample Rate: %.2fHz", (gyroRate * 1000.0f) / (sampleDiv + 1));
}
void setIMUSensitivity(uint8_t accelRange, uint8_t gyroRange, uint8_t accelBandwidth, uint8_t gyroSampleRate)
{
    // ESP_LOGI(TAG, "setIMUSensitivity %d %d %d %d", accelRange, gyroRange, accelBandwidth, gyroSampleRate);
    mpu.setFullScaleAccelRange(accelRange);
    mpu.setFullScaleGyroRange(gyroRange);
    // 待定
    // mpu.setRate()
    // mpu.setDHPFMode()
}

//饿狗重启
void esp_task_wdt_isr_user_handler(void)
{
    reset(true);
}

// 自动校准mpu6050
void calibrateIMU()
{
    enableIMU();
    setIMUSensitivity(2, 3, 1, 1);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    //老外的这个校准函数有点问题，经常会卡住，所以开启任务看门狗。
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    // Calibration Time: generate offsets and calibrate our MPU6050
    printf("CalibrateAccel...\n");
    mpu.CalibrateAccel(6);
    printf("CalibrateGyro...\n");
    mpu.CalibrateGyro(6);

    esp_task_wdt_reset();
    //关闭看门狗
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));

    int16_t *mpuoffset = mpu.GetActiveOffsets();
    printf("accl x,y,z offset : %6d %6d %6d\n", mpuoffset[0], mpuoffset[1], mpuoffset[2]);
    printf("gyro x,y,z offset : %6d %6d %6d\n", mpuoffset[3], mpuoffset[4], mpuoffset[5]);

    printf("saving...\n");
    saveIMUCaliOffset(mpuoffset);
    int16_t acclx = 0, accly = 0, acclz = 0;
    int16_t gyrox = 0, gyroy = 0, gyroz = 0;
    loadIMUCaliOffset(&acclx, &accly, &acclz, &gyrox, &gyroy, &gyroz);

    if (mpuoffset[0] == acclx && mpuoffset[1] == accly && mpuoffset[2] == acclz &&
        mpuoffset[3] == gyrox && mpuoffset[4] == gyroy && mpuoffset[5] == gyroz)
    {
        printf("save success!\n");
    }
    else
    {
        printf("save failed:\n");
        printf("The saved value: %6d %6d %6d %6d %d %6d is not match!\n",
               acclx, accly, acclz, gyrox, gyroy, gyroz);
    }
}

void getIMUMotionData(int16_t outData[6])
{
    mpu.getMotion6(&outData[0], &outData[1], &outData[2], &outData[3], &outData[4], &outData[5]);
}

static MyMeanFilter _mBatteryVoltage;
static int _vs[5];

static void _measureBatt()
{
    int v = getADCValueAfterCali(PIN_ADC_BATT_LEVEL) * BATT_VOLTAGE_MULTIPLY_FACTOR;
    MyMeanFilterUpdate(&_mBatteryVoltage, v);
}

static void updateMeasureBatt(uint32_t dt)
{
    static uint32_t t_notify = 0;
    static uint32_t t_measure = 0;

    t_measure += dt;
    // 5秒检测一次电池电量
    if (t_measure >= 5 * 1000)
    {
        _measureBatt();
        t_measure = 0;
    }

    t_notify += dt;
    //30秒发送一次电池电量
    if (t_notify >= 30 * 1000)
    {
        pGamepad->notifyBattLevel(getBatteryLevel());
        t_notify = 0;
    }
}

void initADCBattery()
{
    _initADCHandle();
    adc_oneshot_chan_cfg_t chanCfg = {
        .atten = DEFAULT_ADC_ATTEN_DB,
        .bitwidth = DEFAULT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, PIN_ADC_BATT_LEVEL, &chanCfg));

    adc_cali_curve_fitting_config_t caliCfg = {
        .unit_id = DEFAULT_ADC_UNIT_ID,
        .chan = PIN_ADC_BATT_LEVEL,
        .atten = DEFAULT_ADC_ATTEN_DB,
        .bitwidth = DEFAULT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &adcCaliHandle));

    gpio_reset_pin(PIN_CHARGING_STATE);
    gpio_set_direction(PIN_CHARGING_STATE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_CHARGING_STATE, GPIO_FLOATING);
    gpio_reset_pin(PIN_CHARGING_STDBY_STATE);
    gpio_set_direction(PIN_CHARGING_STDBY_STATE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_CHARGING_STDBY_STATE, GPIO_FLOATING);

    MyMeanFilterInit(&_mBatteryVoltage, _vs, sizeof(_vs));
    for (size_t i = 0; i < sizeof(_vs); i++)
    {
        _measureBatt();
    }
}

int getBatteryVoltage()
{
    return (int)round(_mBatteryVoltage.mean);
}
int getBatteryLevel()
{
    int v = clamp(getBatteryVoltage(), 3300, 4200) - 3300;
    int vv = v / (42 - 33);
    return vv;
}

//检测是否充电中。不是很准，TP4056芯片返回的值偶尔会抖动一下，不知为啥。
int isBatteryCharging()
{
    return (!gpio_get_level(PIN_CHARGING_STATE)) || (!isPowOnByPushBtn);
}

int isBatteryChargeFull()
{
    return !gpio_get_level(PIN_CHARGING_STDBY_STATE);
}

void setLEDScanning(uint8_t isScanning)
{
    _isScanning = isScanning;
}

void initLED()
{
    gpio_reset_pin(PIN_LED_1);
    gpio_reset_pin(PIN_LED_2);
    gpio_reset_pin(PIN_LED_3);
    gpio_reset_pin(PIN_LED_4);
    gpio_set_direction(PIN_LED_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_4, GPIO_MODE_OUTPUT);
}

void initPowSave(bool enableLightSleep)
{
    esp_pm_config_t cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = 40,
        .light_sleep_enable = enableLightSleep};
    ESP_ERROR_CHECK(esp_pm_configure(&cfg));
}

void onlyChargeMode()
{
    ESP_LOGI(TAG, "Just charging, not working!");
    initLED();

    esp_wifi_bt_power_domain_off();
    initPowSave(true);

    //临时接上电池，用于测量电池电压。
    connectBattery();
    DELAY(100);
    initADCBattery();
    disconnectBattery();

    int level = getBatteryLevel();
    uint8_t state2[] = {0b0000, 0b0100, 0b1100, 0b1101, 0b1111};

    int64_t t0 = esp_timer_get_time();
    uint32_t t_measBatt = 0;
    uint32_t t_showingBatt = 0;
    uint32_t levelFlashing = 0;
    while (1)
    {
        uint32_t dt = (esp_timer_get_time() - t0) / 1000;
        t0 = esp_timer_get_time();
        //仅充电模式中，按下xbox键直接重启。
        int isXboxBtnDown = !gpio_get_level(PIN_BTN_XBOX);
        if (isXboxBtnDown)
        {
            reset(false);
        }

        t_measBatt += dt;
        if (t_measBatt > (30 * 1000 - 100))
        {
            //提前100ms接通电池
            connectBattery();
        }
        if (t_measBatt > (30 * 1000))
        {
            //每隔30秒测一次电压
            _measureBatt();
            level = getBatteryLevel();
            t_measBatt = 0;
            ESP_LOGI(TAG, "battery level: %d", level);
            //测完断开电池
            disconnectBattery();
        }

        t_showingBatt += dt;
        if (t_showingBatt < ((150) * sizeof(state2)))
        {
            //LED电量滚动效果
            int i = (t_showingBatt / (150)) % sizeof(state2);
            setLEDState(state2[i]);
        }
        else
        {
            //电量<95LED闪烁效果
            if (level < 95 && !isBatteryChargeFull())
            {
                levelFlashing += dt;
                int i = level / 25;
                if ((levelFlashing / (150)) % 2)
                    setLEDState(state2[i + 1]);
                else
                    setLEDState(state2[i]);
            }
            //满电不闪烁
            else
            {
                setLEDState(state2[sizeof(state2) - 1]);
            }
        }
        DELAY(30);
    }
}

void initBtnSelect()
{
    gpio_reset_pin(PIN_BTN_SELECT);
    gpio_set_direction(PIN_BTN_SELECT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BTN_SELECT, GPIO_PULLUP_ONLY);
}
int isBtnSelectPress()
{
    return !gpio_get_level(PIN_BTN_SELECT);
}
void initNVS()
{
    ESP_ERROR_CHECK(nvs_flash_init());
}
Enum_Gamepad_Type loadGamepadType()
{
    nvs_handle_t h;
    uint8_t t = 0;
    nvs_open(mynvs, NVS_READWRITE, &h);
    nvs_get_u8(h, mynvsKeyGPT, &t);
    nvs_close(h);
    return (Enum_Gamepad_Type)t;
}
void saveGamepadType(Enum_Gamepad_Type t)
{
    nvs_handle_t h;
    uint8_t tt = 0;
    nvs_open(mynvs, NVS_READWRITE, &h);
    nvs_get_u8(h, mynvsKeyGPT, &tt);
    if (tt != (uint8_t)t)
    {
        nvs_set_u8(h, mynvsKeyGPT, (uint8_t)t);
        nvs_commit(h);
    }
    nvs_close(h);
}
static int _ledShowBattLevel = 0;
void updateLED(uint32_t dt)
{
    static uint32_t t_showingBatt = 0;
    static uint32_t t_scanning = 0;
    static uint32_t t_measBatt = 0;
    uint8_t state[] = {0b0001, 0b1011, 0b0101, 0b0000, 0b0000};
    uint8_t state2[] = {0b0000, 0b0100, 0b1100, 0b1101, 0b1111};

    if (_isScanning)
    {
        t_scanning += dt;
        if ((t_scanning) > (DEFAULT_SCAN_TIME_OUT_MS))
        {
            //扫描超时自动关机
            powOff();
            return;
        }
        //LED扫描灯效
        int i = (t_scanning / (100)) % sizeof(state);
        setLEDState((state[i] << (uint8_t)_gamepadType) | state[i] >> (4 - (uint8_t)_gamepadType));
    }
    else
    {
        t_scanning = 0;

        static int level = getBatteryLevel();
        t_measBatt += dt;
        //30s更新一次电量
        if (t_measBatt > 30 * 1000)
        {
            level = getBatteryLevel();
            t_measBatt = 0;
            ESP_LOGI(TAG, "battery level: %d", level);
        }

        static uint32_t levelFlashing = 0;
        if (t_showingBatt == 0 && _ledShowBattLevel)
        {
            t_showingBatt = 1;
            levelFlashing = 0;
        }
        //单击xbox键 或者 电量<20 LED显示电量
        if (t_showingBatt != 0 || level < 20)
        {
            t_showingBatt += dt;
            uint32_t t0 = ((150) * sizeof(state2));
            if (t_showingBatt < t0)
            {
                //LED电量滚动效果
                int i = (t_showingBatt / (150)) % sizeof(state2);
                setLEDState(state2[i]);
            }
            else if (t_showingBatt < t0 + (3 * 1000))
            {
                //电量<95LED闪烁效果
                if (level < 95 && !isBatteryChargeFull())
                {
                    levelFlashing += dt;
                    int i = level / 25;
                    if ((levelFlashing / (150)) % 2)
                        setLEDState(state2[i + 1]);
                    else
                        setLEDState(state2[i]);
                }
                //满电不闪烁
                else
                {
                    setLEDState(state2[sizeof(state2) - 1]);
                }
            }
            else
            {
                t_showingBatt = 0;
            }
        }
        else
        {
            //LED显示手柄类型
            setLEDState(0b0001 << (uint8_t)_gamepadType);
        }
    }
}
static void trySaveSticksValue(char *args[12])
{
    int minx = atoi(args[0]);
    int midx = atoi(args[1]);
    int maxx = atoi(args[2]);
    int miny = atoi(args[3]);
    int midy = atoi(args[4]);
    int maxy = atoi(args[5]);
    int minz = atoi(args[6]);
    int midz = atoi(args[7]);
    int maxz = atoi(args[8]);
    int minrz = atoi(args[9]);
    int midrz = atoi(args[10]);
    int maxrz = atoi(args[11]);
    printf("saving...: %d %d %d %d %d %d %d %d %d %d %d %d\n", minx, midx, maxx, miny, midy, maxy, minz, midz, maxz, minrz, midrz, maxrz);
    saveSticksOffset(minx, midx, maxx, miny, midy, maxy, minz, midz, maxz, minrz, midrz, maxrz);
    loadSticksOffset(&min_x, &mid_x, &max_x, &min_y, &mid_y, &max_y, &min_z, &mid_z, &max_z, &min_rz, &mid_rz, &max_rz);
    if (minx == min_x && midx == mid_x && maxx == max_x &&
        miny == min_y && midy == mid_y && maxy == max_y &&
        minz == min_z && midz == mid_z && maxz == max_z &&
        minrz == min_rz && midrz == mid_rz && maxrz == max_rz)
    {
        printf("save success!\n");
    }
    else
    {
        printf("save failed:\n");
        printf("The saved value: %d %d %d %d %d %d %d %d %d %d %d %d is not match!\n",
               min_x, mid_x, max_x, min_y, mid_y, max_y, min_z, mid_z, max_z, min_rz, mid_rz, max_rz);
    }
}
static void trySaveDeadZone(char *args[4])
{
    int deadx = atoi(args[0]);
    int deady = atoi(args[1]);
    int deadz = atoi(args[2]);
    int deadrz = atoi(args[3]);
    printf("saving...: %d %d %d %d\n", deadx, deady, deadz, deadrz);
    saveDeadZone(deadx, deady, deadz, deadrz);
    loadDeadZone(&dead_x, &dead_y, &dead_z, &dead_rz);
    if (deadx == dead_x && deady == dead_y && deadz == dead_z && deadrz == dead_rz)
    {
        printf("save success!\n");
    }
    else
    {
        printf("save failed:\n");
        printf("The saved value: %d %d %d %d is not match!\n",
               dead_x, dead_y, dead_z, dead_rz);
    }
}
static bool calibratingSticks = false;
static void parseMsg(char *msg)
{
    printf("%s\n", msg); //printf没换行它居然不立即输出。
    char *args[16];
    int argc = esp_console_split_argv(msg, args, 16);

    char *cmd = args[0];
    if (0 == strcmp(cmd, cmdCaliMPU))
    {
        calibrateIMU();
    }
    else if (0 == strcmp(cmd, cmdCaliSticks))
    {
        calibratingSticks = !calibratingSticks;
    }
    else if (0 == strcmp(cmd, cmdSaveSticks))
    {
        if (argc < 13)
        {
            printf("需要12个参数！！！\n");
            return;
        }
        trySaveSticksValue(&args[1]);
    }
    else if (0 == strcmp(cmd, cmdSaveDeadZone))
    {
        if (argc < 5)
        {
            printf("需要4个参数！！！\n");
            return;
        }
        trySaveDeadZone(&args[1]);
    }
}

static void processSerial(uint32_t dt)
{
    static char buf[64] = {};
    static int bufInd = 0;
    static uint32_t t = 0;

    bool hasOneLineMsg = false;
    while (1)
    {
        int c = fgetc(stdin);
        if (c == -1)
            break;
        t = 0;
        //遇到回车换行，算一条信息
        if (c == '\r' || c == '\n')
        {
            hasOneLineMsg = true;
            break;
        }

        buf[bufInd] = c;
        bufInd++;

        //遇到buf满了，算一条信息。
        if (bufInd >= (sizeof(buf) - 1))
        {
            hasOneLineMsg = true;
            break;
        }
    }

    if (!hasOneLineMsg)
    {
        //超过30ms，也算一条完整的信息。
        t += dt;
        if (t > 30)
        {
            t = 0;
            hasOneLineMsg = true;
        }
    }

    if (hasOneLineMsg && bufInd > 0)
    {
        buf[bufInd] = 0;

        parseMsg(buf);

        bufInd = 0;
        buf[0] = 0;
    }
}

void workMode()
{
    ESP_LOGI(TAG, "%s run in core %d!", __func__, esp_cpu_get_core_id());

    initMyKeypadMatrix(PIN_ROW_0, PIN_ROW_1, PIN_ROW_2, PIN_ROW_3, PIN_COL_0, PIN_COL_1, PIN_COL_2, PIN_COL_3);
    initNVS();

    bool clearBond = false;
    bool calibrationMode = false;
    bool powBoostMode = false;

    uint16_t ks = scanMyKeyMatrix(BTN_PIN_LEVEL_STABLE_DELAY_NOP_COUNT);
    KeyMatrix_t *keys = (KeyMatrix_t *)&ks;
    printMyKeypadMatrix(TAG, ks, 1);

    //xbox模式
    if (keys->X)
    {
        _gamepadType = GAMEPAD_TYPE_XBOX;
        saveGamepadType(_gamepadType);
    }
    //switch pro模式
    else if (keys->A)
    {
        _gamepadType = GAMEPAD_TYPE_SWITCH_PRO;
        saveGamepadType(_gamepadType);
    }
    //dual sense模式
    else if (keys->Y)
    {
        _gamepadType = GAMEPAD_TYPE_DS;
        saveGamepadType(_gamepadType);
    }
    //校准模式
    else if (keys->B)
    {
        calibrationMode = true;
    }
    //上次打开的模式
    else
    {
        _gamepadType = loadGamepadType();
    }

    if (!calibrationMode)
    {
        //清空绑定
        if (keys->Start)
        {
            clearBond = true;
        }
        if (keys->LB)
        {
            powBoostMode = true;
        }
    }

    initBtnSelect();
    initLED();
    initADCSticks();
    initADCBattery();

    initMotor();
    initI2C();

    min_x = 0;
    mid_x = 50;
    max_x = 100;
    min_y = 0;
    mid_y = 50;
    max_y = 100;
    min_z = 0;
    mid_z = 50;
    max_z = 100;
    min_rz = 0;
    mid_rz = 50;
    max_rz = 100;
    loadSticksOffset(&min_x, &mid_x, &max_x, &min_y, &mid_y, &max_y, &min_z, &mid_z, &max_z, &min_rz, &mid_rz, &max_rz);
    ESP_LOGI(TAG, "load sticks range value: %d %d %d %d %d %d %d %d %d %d %d %d", min_x, mid_x, max_x, min_y, mid_y, max_y, min_z, mid_z, max_z, min_rz, mid_rz, max_rz);

    dead_x = 0;
    dead_y = 0;
    dead_z = 0;
    dead_rz = 0;
    loadDeadZone(&dead_x, &dead_y, &dead_z, &dead_rz);
    ESP_LOGI(TAG, "load deadzone value: %d %d %d %d", dead_x, dead_y, dead_z, dead_rz);

    if (!calibrationMode)
    {
        initGamepadBLE(clearBond, powBoostMode);
        //爆pow模式，抖一下
        if (powBoostMode)
        {
            setMotorLevel(100, 100);
            DELAY(200);
            setMotorLevel(0, 0);
        }
        else
        {
        }
        esp_sleep_enable_bt_wakeup();
        initPowSave(ENABLE_LIGHT_SLEEP);

        int64_t lastT = esp_timer_get_time();

        while (1)
        {

            int64_t t = esp_timer_get_time();
            uint32_t dt = (t - lastT) / 1000; //ms
            lastT = t;

            uint8_t btnXbox = checkXboxBtn(dt);
            uint8_t btnSelect = isBtnSelectPress();

            //xbox被按住不放，依旧处于通电状态。
            if (_isPowOff)
            {
                DELAY(15);
                continue;
            }

            ks = scanMyKeyMatrix(BTN_PIN_LEVEL_STABLE_DELAY_NOP_COUNT);
            keys = (KeyMatrix_t *)&ks;
            // printMyKeypadMatrix(TAG, ks, 1);

            int x, y, z, rz, lt, rt;
            x = getADCValue(PIN_ADC_X);
            y = getADCValue(PIN_ADC_Y);
            z = getADCValue(PIN_ADC_Z);
            rz = getADCValue(PIN_ADC_RZ);
            lt = getADCValue(PIN_ADC_LT);
            rt = getADCValue(PIN_ADC_RT);

            normalizeJoyStrickValue(&x, &y, &z, &rz);
            normalizeTriggerValue(&lt, &rt);

            updateMeasureBatt(dt);
            //单击xbox键显示电量
            _ledShowBattLevel = btnXbox;
            updateLED(dt);

            if (pGamepad->getConnectedCount())
            {
                pGamepad->update(dt, keys, btnXbox, btnSelect, x, y, z, rz, lt, rt);
            }
            else
            {
                DELAY(15);
            }
        }
    }
    else
    {
        disconnectBattery();

        printf("\n"
               "校准模式帮助：\n"
               "1.陀螺仪校准：把手柄静置于一水平面，然后在串口输入 %s 将自动校准保存。\n"
               "2.摇杆校准：\n"
               "2.1.串口输入 %s 可以切换串口是否输出当前摇杆值。\n"
               "2.2.不动摇杆，观察 x y z rz 输出的值，此值为各方向摇杆的中间值，自己记下来。\n"
               "2.3.转到摇杆，观察 x y z rz 输出值的变化，自己记录下各个方向的最大最小值。\n"
               "2.4.串口输入 /s x_min x_mid x_max y_min y_mid y_max z_min z_mid z_max rz_min rz_mid rz_max \n"
               "(x y z rz 对应摇杆4个方向，min最小值，mid中间值，max最大值；)\n"
               "(如：%s 19 50 87 5 48 83 0 52 90 0 51 86)\n"
               "(该过程将保存摇杆校准的数据到手柄中。)\n"
               "3.设置摇杆死区：在串口输入 %s dead_x dead_y dead_z dead_rz \n"
               "(dead_x dead_y dead_z dead_rz 对应摇杆4个方向死区值，通常是填摇杆不动时中间值的变化幅度)\n"
               "(注意此处死区的设置仅用于判断手柄输入是否变化，对于轮询输入是手柄没任何效果)\n",
               cmdCaliMPU, cmdCaliSticks, cmdSaveSticks, cmdSaveDeadZone);
        uint8_t state[] = {0b0001, 0b0010, 0b0100, 0b1000};
        int64_t lastT = esp_timer_get_time();

        while (1)
        {

            int64_t t = esp_timer_get_time();
            uint32_t dt = (t - lastT) / 1000; //ms
            lastT = t;

            (void)checkXboxBtn(dt);
            // uint8_t btnSelect = isBtnSelectPress();

            //xbox被按住不放，依旧处于通电状态。
            if (_isPowOff)
            {
                DELAY(15);
                continue;
            }

            ks = scanMyKeyMatrix(BTN_PIN_LEVEL_STABLE_DELAY_NOP_COUNT);
            keys = (KeyMatrix_t *)&ks;
            // printMyKeypadMatrix(TAG, ks, 1);

            int x, y, z, rz; // lt, rt;
            x = getADCValue(PIN_ADC_X);
            y = getADCValue(PIN_ADC_Y);
            z = getADCValue(PIN_ADC_Z);
            rz = getADCValue(PIN_ADC_RZ);
            // lt = getADCValue(PIN_ADC_LT);
            // rt = getADCValue(PIN_ADC_RT);

            if (calibratingSticks)
            {
                printf("x y z rz: %3d %3d %3d %3d\n",
                       (int)(((4095 - x) * 100L) / 4095),
                       (int)((y * 100L) / 4095),
                       (int)(((4095 - z) * 100L) / 4095),
                       (int)(((4095 - rz) * 100L) / 4095));
            }

            int i = (t / (100 * 1000)) % 4;
            setLEDState(state[i]);

            processSerial(dt);

            DELAY(15);
        }
    }
}

void app_main(void)
{

    isPowOnByPushBtn = isBatteryConnected();
    ESP_LOGI(TAG, "------------isPowOnByPushBtn: %d------------", isPowOnByPushBtn);
    //禁用睡眠时切换gpio状态
    esp_sleep_enable_gpio_switch(false);

    if (isPowOnByPushBtn)
    {
        powOn();
        workMode();
    }
    else
    {
        disconnectBattery();
        onlyChargeMode();
    }
}
