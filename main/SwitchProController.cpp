/**
 * 参考：
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering.git
 * https://github.com/mumumusuc/libjoycon.git
 * https://github.com/EasyConNS/BlueCon-esp32.git
 * SDL-SDL2\src\joystick\hidapi\SDL_hidapi_switch.c
*/
/**
 * steam 有点问题，如果先开steam再连手柄会出现手柄没振动功能；
 * 先连手柄再开steam就没问题。
*/
#include "SwitchProController.h"
#include "NimBLEDevice.h"
#include "ble_uuid.h"
#include "helper.h"
#include "math.h"

#define VID_SIG 2 //(1=Bluetooth SIG, 2=USB SIG)
#define VID 0x057E
#define PID 0x2009
#define PID_VER 0x0101

#define HID_RPT_ID_INPUT_STD 0x21           //Standard input reports used for subcommand replies.
#define HID_RPT_ID_INPUT_NFC_FW_UPDATE 0x23 //NFC/IR MCU FW update input report.
#define HID_RPT_ID_INPUT_FULL 0x30          //input reports with IMU data instead of subcommand replies.Pushes current state @120Hz
#define HID_RPT_ID_INPUT_NFC_MCU 0x31       //Pushes large packets with standard input report + NFC/IR MCU data input report.
#define HID_RPT_ID_INPUT_UNKNOWN1 0x32      //Sends standard input reports.
#define HID_RPT_ID_INPUT_UNKNOWN2 0x33      //Sends standard input reports.
#define HID_RPT_ID_INPUT 0x3F

#define HID_RPT_ID_OUTPUT_RUMBLE_SUBCMD 0x01
#define HID_RPT_ID_OUTPUT_RUMBLE 0x10
#define HID_RPT_ID_OUTPUT_REQ_NFC_OR_RUMBLE 0x11
#define HID_RPT_ID_OUTPUT_UNKNOWN 0x12

#define SUB_CMD_GET_STATE 0x00
#define SUB_CMD_BT_MANUAL_PAIRING 0x01
#define SUB_CMD_REQ_DEVICE_INFO 0x02
#define SUB_CMD_SET_INPUT_RPT_MODE 0x03
#define SUB_CMD_TRIGGER_BTNS_TIME_ELAPSED 0x04
#define SUB_CMD_GET_PAGE_LIST_STATE 0x05
#define SUB_CMD_SET_HCI_STATE 0x06
#define SUB_CMD_RESET_PAIRING_INFO 0x07
#define SUB_CMD_SET_LOW_POWER_SHIPPING_STATE 0x08
#define SUB_CMD_SET_SPI_FLASH_READ 0x10
#define SUB_CMD_SET_SPI_FLASH_WRITE 0x11
#define SUB_CMD_SET_SPI_SECTOR_ERASE 0x12
#define SUB_CMD_SET_RESET_NFC 0x20
#define SUB_CMD_SET_SET_NFC_CONFIG 0x21
#define SUB_CMD_SET_SET_NFC_STATE 0x22
#define SUB_CMD_SET_PLAYER_LED 0x30
#define SUB_CMD_GET_PLAYER_LED 0x31
#define SUB_CMD_SET_HOME_LED 0x38
#define SUB_CMD_ENABLE_IMU 0x40
#define SUB_CMD_SET_IMU_SENSITIVITY 0x41
#define SUB_CMD_WRITE_IMU_REG 0x42
#define SUB_CMD_READ_IMU_REG 0x43
#define SUB_CMD_ENABLE_VIBRATION 0x48
#define SUB_CMD_GET_REGULATED_VOLTAGE 0x50

#define INPUT_REPORT_FULL_SIZE 48
#define INPUT_REPORT_STAND_SIZE 48

#define RPT_MODE_NFC_IR_CAM 0x0  //Active polling for NFC/IR camera data.
#define RPT_MODE_NFC_IR_MCU 0x1  //Active polling mode for NFC/IR MCU configuration data
#define RPT_MODE_NFC_IR_DATA 0x2 //Active polling mode for NFC/IR data and configuration
#define RPT_MODE_IR_CAM 0x3      //Active polling mode for IR camera data.
#define RPT_MODE_STANDARD 0x30   //Standard full mode. Pushes current state @60Hz
#define RPT_MODE_NFC_IR 0x31     //NFC/IR mode. Pushes large packets @60Hz
#define RPT_MODE_33 0x33         // ?
#define RPT_MODE_35 0x35         // ?
#define RPT_MODE_SIMPLE_HID 0x3F //Simple HID mode. Pushes updates with every button press

#define DELAY(x) vTaskDelay((x) / portTICK_PERIOD_MS)

static const char *TAG = "swpro";

static const uint8_t reportmap[] = {
    0x05, 0x01,                                // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,                                // Usage (Game Pad)
    0xA1, 0x01,                                // Collection (Application)
    0x06, 0x01, 0xFF,                          //   Usage Page (Vendor Defined 0xFF01)
    0x85, HID_RPT_ID_INPUT_STD,                //   Report ID (33)
    0x09, 0x21,                                //   Usage (0x21)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, HID_RPT_ID_INPUT_FULL,               //   Report ID (48)
    0x09, 0x30,                                //   Usage (0x30)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, HID_RPT_ID_INPUT_NFC_MCU,            //   Report ID (49)
    0x09, 0x31,                                //   Usage (0x31)
    0x75, 0x08,                                //   Report Size (8)
    0x96, 0x69, 0x01,                          //   Report Count (361)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, HID_RPT_ID_INPUT_UNKNOWN1,           //   Report ID (50)
    0x09, 0x32,                                //   Usage (0x32)
    0x75, 0x08,                                //   Report Size (8)
    0x96, 0x69, 0x01,                          //   Report Count (361)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, HID_RPT_ID_INPUT_UNKNOWN2,           //   Report ID (51)
    0x09, 0x33,                                //   Usage (0x33)
    0x75, 0x08,                                //   Report Size (8)
    0x96, 0x69, 0x01,                          //   Report Count (361)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, HID_RPT_ID_INPUT,                    //   Report ID (63)
    0x05, 0x09,                                //   Usage Page (Button)
    0x19, 0x01,                                //   Usage Minimum (0x01)
    0x29, 0x10,                                //   Usage Maximum (0x10)
    0x15, 0x00,                                //   Logical Minimum (0)
    0x25, 0x01,                                //   Logical Maximum (1)
    0x75, 0x01,                                //   Report Size (1)
    0x95, 0x10,                                //   Report Count (16)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,                                //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,                                //   Usage (Hat switch)
    0x15, 0x00,                                //   Logical Minimum (0)
    0x25, 0x07,                                //   Logical Maximum (7)
    0x75, 0x04,                                //   Report Size (4)
    0x95, 0x01,                                //   Report Count (1)
    0x81, 0x42,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x05, 0x09,                                //   Usage Page (Button)
    0x75, 0x04,                                //   Report Size (4)
    0x95, 0x01,                                //   Report Count (1)
    0x81, 0x01,                                //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,                                //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,                                //   Usage (X)
    0x09, 0x31,                                //   Usage (Y)
    0x09, 0x33,                                //   Usage (Rx)
    0x09, 0x34,                                //   Usage (Ry)
    0x16, 0x00, 0x00,                          //   Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,              //   Logical Maximum (65534)
    0x75, 0x10,                                //   Report Size (16)
    0x95, 0x04,                                //   Report Count (4)
    0x81, 0x02,                                //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x01, 0xFF,                          //   Usage Page (Vendor Defined 0xFF01)
    0x85, HID_RPT_ID_OUTPUT_RUMBLE_SUBCMD,     //   Report ID (1)
    0x09, 0x01,                                //   Usage (0x01)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x91, 0x02,                                //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, HID_RPT_ID_OUTPUT_RUMBLE,            //   Report ID (16)
    0x09, 0x10,                                //   Usage (0x10)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x91, 0x02,                                //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, HID_RPT_ID_OUTPUT_REQ_NFC_OR_RUMBLE, //   Report ID (17)
    0x09, 0x11,                                //   Usage (0x11)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x91, 0x02,                                //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, HID_RPT_ID_OUTPUT_UNKNOWN,           //   Report ID (18)
    0x09, 0x12,                                //   Usage (0x12)
    0x75, 0x08,                                //   Report Size (8)
    0x95, 0x30,                                //   Report Count (48)
    0x91, 0x02,                                //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,                                      // End Collection
};
//0x6020~0x604E
static const uint8_t factoryCalibration[] = {
    //20~37 6-Axis motion sensor Factory calibration
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Acc XYZ origin position when completely horizontal and stick is upside
    0x00, 0x40, 0x00, 0x40, 0x00, 0x40, //Acc XYZ sensitivity special coeff, for default sensitivity: ±8G.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Gyro XYZ origin position when still
    0x3b, 0x34, 0x3b, 0x34, 0x3b, 0x34, //Gyro XYZ sensitivity special coeff, for default sensitivity: ±2000dps
    //38~3C
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    ///3D~4E left right stick calibration
    0xFF, 0xF7, 0x7F, //x above center, y above center: 0x7FF 0x7FF
    0xFF, 0xF7, 0x7F, //x center, y center: 0x7FF
    0xFF, 0xF7, 0x7F, //x below center, y below center: 0x7FF 0x7FF
    0xFF, 0xF7, 0x7F, //x above center, y above center: 0x7FF 0x7FF
    0xFF, 0xF7, 0x7F, //x center, y center: 0x7FF
    0xFF, 0xF7, 0x7F, //x below center, y below center: 0x7FF 0x7FF
};
//0x8010~0x8025 sticks calibration, 0x8026~0x803F motion sensor calibration
static uint8_t userCalibration[] = {
    0xB2, 0xA1,                         //left stick
    0xFF, 0xF7, 0x7F,                   //x above center, y above center: 0x7FF 0x7FF
    0xFF, 0xF7, 0x7F,                   //x center, y center: 0x7FF
    0xFF, 0xF7, 0x7F,                   //x below center, y below center: 0x7FF 0x7FF
    0xB2, 0xA1,                         //right stick
    0xFF, 0xF7, 0x7F,                   //x above center, y above center: 0x7FF 0x7FF
    0xFF, 0xF7, 0x7F,                   //x center, y center: 0x7FF
    0xFF, 0xF7, 0x7F,                   //x below center, y below center: 0x7FF 0x7FF
    0xB2, 0xA1,                         //Motion sensor calibration
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Acc XYZ origin position when completely horizontal and stick is upside
    0x00, 0x40, 0x00, 0x40, 0x00, 0x40, //Acc XYZ sensitivity special coeff, for default sensitivity: ±8G.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Gyro XYZ origin position when still
    0x3b, 0x34, 0x3b, 0x34, 0x3b, 0x34, //Gyro XYZ sensitivity special coeff, for default sensitivity: ±2000dps
};

static void copyCalibration2Buf(uint16_t addr, uint8_t *buf, int16_t len)
{
    memset(buf, 0xFF, len);
    int16_t _addr;
    int16_t n;
    const uint8_t *calibration;
    if (addr & 0x8000)
    {
        _addr = addr - 0x8010;
        n = sizeof(userCalibration);
        calibration = userCalibration;
    }
    else
    {
        _addr = addr - 0x6020;
        n = sizeof(factoryCalibration);
        calibration = factoryCalibration;
    }
    if (_addr >= n)
        return;
    if ((_addr + len) <= 0)
        return;

    int16_t bufind0 = 0;
    int16_t startind = _addr;
    if (_addr < 0)
    {
        bufind0 = -_addr;
        startind = 0;
    }
    int16_t len0 = len - bufind0 + startind;
    len = len0 < len ? len0 : len;
    memcpy(&buf[bufind0], &calibration[startind], len);
}

static Input_Report ipacket;

SwitchProController::SwitchProController(const std::string &deviceName) : IMyGamepad(deviceName)
{
    ipRptMode = HID_RPT_ID_INPUT;

    // ipRptMode = HID_RPT_ID_INPUT_FULL;

    int isNRpa;
    ble_hs_id_copy_addr(0, mac, &isNRpa);
    ESP_LOGI(TAG, "BLE Public Addr: %02x:%02x:%02x:%02x:%02x:%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    pHidDev->pnp(VID_SIG, BUILD_UINT16(HI_UINT16(VID), LO_UINT16(VID)), BUILD_UINT16(HI_UINT16(PID), LO_UINT16(PID)), BUILD_UINT16(HI_UINT16(PID_VER), LO_UINT16(PID_VER)));
    pHidDev->hidInfo(0x00, 0x01);
    pHidDev->reportMap((uint8_t *)reportmap, sizeof(reportmap));

    _iRptChara = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT);
    _iRptChara->setCallbacks(this);
    _iRptChara_STD = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_STD);
    _iRptChara_STD->setCallbacks(this);
    _iRptChara_Full = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_FULL);
    _iRptChara_Full->setCallbacks(this);
    _iRptChara_NFC_MCU = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_NFC_MCU);
    _iRptChara_NFC_MCU->setCallbacks(this);
    _iRptChara_Unknown1 = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_UNKNOWN1);
    _iRptChara_Unknown1->setCallbacks(this);
    _iRptChara_Unknown2 = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_UNKNOWN2);
    _iRptChara_Unknown2->setCallbacks(this);

    _oRptChara_Rumble_SubCMD = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_RUMBLE_SUBCMD);
    _oRptChara_Rumble_SubCMD->setCallbacks(this);
    _oRptChara_Rumble = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_RUMBLE);
    _oRptChara_Rumble->setCallbacks(this);
    _oRptChara_ReqNFC_Rumble = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_REQ_NFC_OR_RUMBLE);
    _oRptChara_ReqNFC_Rumble->setCallbacks(this);
    _oRptChara_Unknown = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_UNKNOWN);
    _oRptChara_Unknown->setCallbacks(this);

    _enableIMU(true);
}
SwitchProController::~SwitchProController() {}
bool SwitchProController::isRumbling()
{
    uint16_t l, r;
    getMotorLevel(&l, &r);
    return l > 0 || r > 0;
}
void SwitchProController::setSTDOrFullInputPacket(KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    ipacket.btn.A = keys->B;
    ipacket.btn.B = keys->A;
    ipacket.btn.X = keys->Y;
    ipacket.btn.Y = keys->X;
    ipacket.btn.L = keys->LB;
    ipacket.btn.R = keys->RB;
    ipacket.btn.LS = keys->LS;
    ipacket.btn.RS = keys->RS;
    ipacket.btn.Plus = keys->Start;
    ipacket.btn.Home = btnXbox;
    ipacket.btn.Minus = btnSelect;
    ipacket.btn.ZL = lt > 0;
    ipacket.btn.ZR = rt > 0;
    ipacket.btn.Left = keys->Left;
    ipacket.btn.Right = keys->Right;
    ipacket.btn.Up = keys->Up;
    ipacket.btn.Down = keys->Down;
    float ff;
    uint16_t ii;
    ff = (x * 4095L) / 100;
    ii = (uint16_t)ff;
    ipacket.stick.left_H_LB_8 = ii & 0xFF;
    ipacket.stick.left_H_HB_4 = (ii >> 8) & 0x0F;
    ff = (y * 4095L) / 100;
    ii = 4095 - (uint16_t)ff;
    ipacket.stick.left_V_LB_4 = ii & 0x0F;
    ipacket.stick.left_V_HB_8 = (ii >> 4) & 0xFF;
    ff = (z * 4095L) / 100;
    ii = (uint16_t)ff;
    ipacket.stick.right_H_LB_8 = ii & 0xFF;
    ipacket.stick.right_H_HB_4 = (ii >> 8) & 0x0F;
    ff = (rz * 4095L) / 100;
    ii = 4095 - (uint16_t)ff;
    ipacket.stick.right_V_LB_4 = ii & 0x0F;
    ipacket.stick.right_V_HB_8 = (ii >> 4) & 0xFF;
    ipacket.motorStatus = isRumbling();
    if (getIMUEnabled())
    {
        getIMUMotionData((int16_t *)&ipacket.imuData0);
        DELAY(SEND_PERIOD_MS / 3);
        getIMUMotionData((int16_t *)&ipacket.imuData1);
        DELAY(SEND_PERIOD_MS / 3);
        getIMUMotionData((int16_t *)&ipacket.imuData2);
        DELAY(SEND_PERIOD_MS / 3);
    }
    else
    {
        DELAY(SEND_PERIOD_MS);
    }
}
void SwitchProController::update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    static Gamepad_Input_Pro ip = {};
    // int16_t data[6];
    // getIMUMotionData(data);
    // ESP_LOGI(TAG, "x=%6d,y=%6d,z=%6d,Gx=%6d,Gy=%6d,Gz=%6d", data[0], data[1], data[2], data[3], data[4], data[5]);

    switch (ipRptMode)
    {
    case HID_RPT_ID_INPUT_FULL:
        // ESP_LOGI(TAG, "send full input");
        setSTDOrFullInputPacket(keys, btnXbox, btnSelect, x, y, z, rz, lt, rt);
        packetTimer += 2;
        this->sendPacket(_iRptChara_Full, INPUT_REPORT_FULL_SIZE);
        break;
    case HID_RPT_ID_INPUT:
        if (!this->isKeyStateChange(keys, btnXbox, btnSelect, x, y, z, rz, lt, rt))
        {
            DELAY(SEND_PERIOD_MS);
            break;
        }
        ip.btnA = keys->A;
        ip.btnB = keys->B;
        ip.btnX = keys->X;
        ip.btnY = keys->Y;
        ip.btnL = keys->LB;
        ip.btnR = keys->RB;
        ip.btnLS = keys->LS;
        ip.btnRS = keys->RS;
        ip.btnPlus = keys->Start;
        ip.btnHome = btnXbox;
        ip.btnMinus = btnSelect;
        ip.btnZL = lt > 0;
        ip.btnZR = rt > 0;
        float ff;
        ff = (x * 65535L) / 100;
        ip.x = (uint16_t)ff;
        ff = (y * 65535L) / 100;
        ip.y = (uint16_t)ff;
        ff = (z * 65535L) / 100;
        ip.z = (uint16_t)ff;
        ff = (rz * 65535L) / 100;
        ip.Rz = (uint16_t)ff;

        ip.hat = this->dirKey2DPadValue(keys->Up, keys->Right, keys->Down, keys->Left);
        ip.hat = ip.hat == 0 ? 8 : ip.hat - 1;

        if (_iRptChara->getSubscribedCount())
            _iRptChara->notify((uint8_t *)&ip, sizeof(ip), true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);

        DELAY(SEND_PERIOD_MS);
        break;
    default:
        DELAY(SEND_PERIOD_MS);
        break;
    }
}

void SwitchProController::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    uint8_t id = getRptID(pCharacteristic);
    ESP_LOGI(TAG, "Report 0x%02X: reading!", id);
}
void SwitchProController::_enableIMU(bool enable)
{
    ESP_LOGI(TAG, "IMU %s", enable ? "enable" : "disable");
    if (enable)
    {
        enableIMU();
        setIMUSensitivity(2, 3, 1, 1);
    }
    else
    {
        // disableIMU();
    }
}

void SwitchProController::enableVibration(bool enable)
{
    ESP_LOGI(TAG, "Vibration %s", enable ? "enable" : "disable");
    isRumbleEnable = enable;
}
void SwitchProController::setPlayerLED(const SubCMD_30 *data)
{
    ESP_LOGI(TAG, "setLED %d %d", data->player, data->flashing);
}
void SwitchProController::setHomeLED(const SubCMD_38 *data)
{
    ESP_LOGI(TAG, "setHomeLED %d %d %d %d", data->base_duration, data->pattern_count, data->repeat_count, data->start_intensity);
}
void SwitchProController::setLowPower(bool enable)
{
}
//0.0~1.0
float calcRumbleAmp(const Rumble_Data *rumble)
{
    if (rumble->freq_h == 0 && rumble->freq_h_amp == 0x01 && rumble->freq_l == 0x40 && rumble->freq_l_amp == 0x40)
        return 0;

    // ESP_LOGI(TAG, "Rumble %02x %02x %02x %02x", rumble->freq_h, rumble->freq_h_amp, rumble->freq_l, rumble->freq_l_amp);
    //老外的逆向工程文档看不懂，直接用excel表拟合他提供是数据表格
    float a; //0.0~1.0
    uint8_t ea = rumble->freq_h_amp >> 1;
    if (ea <= 1)
        a = 0.007843f * ea;
    else if (ea <= 14)
        a = 0.0084f * exp(0.1733f * ea);
    else if (ea <= 30)
        a = 0.0587f * exp(0.0433f * ea);
    else
        a = 0.115f * exp(0.0217f * ea);

    // 频率用不上直接注释掉
    // float f;
    // if (rumble->freq_h_amp & 0x01)
    // 	f = 320 * exp(0.0054f * rumble->freq_h);
    // else
    // 	f = 40 * exp(0.0217f * (rumble->freq_l & 0x7F));

    return a;
}

void SwitchProController::handleOutputRptRumble(const Output_Rumble *data)
{
    float lmotor = calcRumbleAmp(&data->rumble.lRumble);
    float rmotor = calcRumbleAmp(&data->rumble.rRumble);
    setMotorLevel((uint8_t)(lmotor * 100), (uint8_t)(rmotor * 100));
}
void SwitchProController::handleOutputRptRumbleSubCMD(const Output_Rumble_SubCMD *subcmdPacket)
{
    uint16_t spiaddr;
    handleOutputRptRumble((Output_Rumble *)subcmdPacket);
    // ESP_LOGI(TAG, "subcmd: 0x%02X", subcmdPacket->subcmd);
    // memset(&ipacket, 0, INPUT_REPORT_STAND_SIZE);
    // ipacket.connInfo = 0x0E;
    // ipacket.charging = 0;
    // ipacket.batteryLevel = 6;
    // ipacket.btns[0] = 0x00;
    // ipacket.btns[1] = 0x00;
    // ipacket.btns[2] = 0x00;
    // ipacket.sticks[0] = 0xFF;
    // ipacket.sticks[1] = 0xF7;
    // ipacket.sticks[2] = 0x7F;
    // ipacket.sticks[3] = 0xFF;
    // ipacket.sticks[4] = 0xF7;
    // ipacket.sticks[5] = 0x7F;
    // ipacket.motorStatus = isRumbling();
    ipacket.subcmdReply = 0x80;
    ipacket.subcmdReplyID = subcmdPacket->subcmd;
    ipacket.data[0] = 0x03;
    uint16_t voltage;

    switch (subcmdPacket->subcmd)
    {
    case SUB_CMD_GET_STATE:
        ESP_LOGI(TAG, "get_state");
        break;
    case SUB_CMD_BT_MANUAL_PAIRING:
        ESP_LOGI(TAG, "bt_manual_pairing");
        // subcmdPacket->subcmd01.cmd
        break;
    case SUB_CMD_REQ_DEVICE_INFO:
        ESP_LOGI(TAG, "req_device_info");
        ipacket.subcmdReply = 0x82;
        ipacket.data[0] = 0x03; //firmware~
        ipacket.data[1] = 0x48; //~version
        ipacket.data[2] = 0x03; //pro controller
        ipacket.data[3] = 0x02; //unknown
        //4~9 mac big endian
        ipacket.data[4] = mac[5];
        ipacket.data[5] = mac[4];
        ipacket.data[6] = mac[3];
        ipacket.data[7] = mac[2];
        ipacket.data[8] = mac[1];
        ipacket.data[9] = mac[0];
        ipacket.data[10] = 0x01; //unknown
        ipacket.data[11] = 0x01; //colors in SPI
        break;
    case SUB_CMD_SET_INPUT_RPT_MODE:
        ESP_LOGI(TAG, "set_input_rpt_mode: 0x%02X", subcmdPacket->subcmd03.rptMode);
        switch (subcmdPacket->subcmd03.rptMode)
        {
        case RPT_MODE_NFC_IR_CAM:
        case RPT_MODE_NFC_IR_MCU:
        case RPT_MODE_NFC_IR_DATA:
        case RPT_MODE_IR_CAM:
            break;
        case RPT_MODE_STANDARD:
            ipRptMode = HID_RPT_ID_INPUT_FULL;
            break;
        case RPT_MODE_NFC_IR:
            break;
        case RPT_MODE_SIMPLE_HID:
            ipRptMode = HID_RPT_ID_INPUT;
            break;
        default:
            break;
        }
        break;
    case SUB_CMD_TRIGGER_BTNS_TIME_ELAPSED:
        ESP_LOGI(TAG, "trigger_btns_time_elapsed");
        // subcmdPacket->subcmd04.time
        break;
    case SUB_CMD_GET_PAGE_LIST_STATE:
        ESP_LOGI(TAG, "get_page_list_state");
        break;
    case SUB_CMD_SET_HCI_STATE: // change power state
        ESP_LOGI(TAG, "set_hci_state");
        // subcmdPacket->subcmd06.hciMode
        break;
    case SUB_CMD_RESET_PAIRING_INFO:
        ESP_LOGI(TAG, "reset_pairing_info");
        break;
    case SUB_CMD_SET_LOW_POWER_SHIPPING_STATE:
        ESP_LOGI(TAG, "set_low_power_shipping_state");
        this->setLowPower(subcmdPacket->subcmd08.enable);
        break;
    case SUB_CMD_SET_SPI_FLASH_READ:
        spiaddr = (uint16_t)subcmdPacket->subcmd10.address;
        ESP_LOGI(TAG, "spi_flash_read: 0x%04X", spiaddr);
        ipacket.subcmdReply = 0x90;
        ipacket.data[0] = subcmdPacket->raw[0];
        ipacket.data[1] = subcmdPacket->raw[1];
        ipacket.data[2] = 0x00;
        ipacket.data[3] = 0x00;
        ipacket.data[4] = subcmdPacket->subcmd10.length;
        copyCalibration2Buf(spiaddr, &ipacket.data[5], subcmdPacket->subcmd10.length);
        // for (size_t i = 0; i < subcmdPacket->subcmd10.length; i++)
        // {
        //     printf("%02x ", ipacket.data[5 + i]);
        // }
        // printf("\r\n");
        break;
    case SUB_CMD_SET_SPI_FLASH_WRITE:
        ESP_LOGI(TAG, "spi_flash_write");
        break;
    case SUB_CMD_SET_SPI_SECTOR_ERASE:
        ESP_LOGI(TAG, "spi_sector_erase");
        break;
    case SUB_CMD_SET_RESET_NFC:
        ESP_LOGI(TAG, "reset_nfc");
        break;
    case SUB_CMD_SET_SET_NFC_CONFIG:
        ESP_LOGI(TAG, "set_nfc_config");
        break;
    case SUB_CMD_SET_SET_NFC_STATE:
        ESP_LOGI(TAG, "set_nfc_state");
        break;
    case 0x2A:
        break;
    case SUB_CMD_SET_PLAYER_LED:
        this->setPlayerLED(&(subcmdPacket->subcmd30));
        break;
    case SUB_CMD_GET_PLAYER_LED:
        ESP_LOGI(TAG, "get_player_LED");
        break;
    case SUB_CMD_SET_HOME_LED:
        this->setHomeLED(&(subcmdPacket->subcmd38));
        break;
    case SUB_CMD_ENABLE_IMU:
        this->_enableIMU(subcmdPacket->subcmd40.enable);
        break;
    case SUB_CMD_SET_IMU_SENSITIVITY:
        ESP_LOGI(TAG, "set_IMU_Sensitivity %d %d %d %d",
                 subcmdPacket->subcmd41.accelRange, subcmdPacket->subcmd41.gyroRange,
                 subcmdPacket->subcmd41.accelBandwidth, subcmdPacket->subcmd41.gyroSampleRate);
        setIMUSensitivity(subcmdPacket->subcmd41.accelRange, subcmdPacket->subcmd41.gyroRange,
                          subcmdPacket->subcmd41.accelBandwidth, subcmdPacket->subcmd41.gyroSampleRate);
        break;
    case SUB_CMD_WRITE_IMU_REG:
        ESP_LOGI(TAG, "write_imu_reg");
        break;
    case SUB_CMD_READ_IMU_REG:
        ESP_LOGI(TAG, "read_imu_reg");
        break;
    case SUB_CMD_ENABLE_VIBRATION:
        enableVibration(subcmdPacket->subcmd48.enable);
        break;
    case SUB_CMD_GET_REGULATED_VOLTAGE:
        ESP_LOGI(TAG, "get_regulated_voltage");
        ipacket.subcmdReply = 0xD0;
        voltage = getBatteryVoltage();
        ipacket.data[0] = lowByte(voltage);
        ipacket.data[1] = highByte(voltage);
        break;
    default:
        ESP_LOGI(TAG, "unknown subcmd: 0x%02X", subcmdPacket->subcmd);
        break;
    }
    // 不确定这个packetNum跟fullInputReport是不是共用的。
    // ipacket.timer = subcmdPacket->packetNum;
    // if (_iRptChara_STD->getSubscribedCount())
    //     _iRptChara_STD->notify((uint8_t *)&ipacket, INPUT_REPORT_STAND_SIZE, true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
    // 暂时先当作共用的吧。
    this->sendPacket(_iRptChara_STD, INPUT_REPORT_STAND_SIZE);
}
void SwitchProController::sendPacket(NimBLECharacteristic *chara, uint16_t len)
{
    // ESP_LOGI(TAG, "send packet %d", packetTimer);
    packetTimer++;
    ipacket.timer = packetTimer;
    ipacket.charging = isBatteryCharging();
    uint16_t v = getBatteryVoltage();
    v = clamp(v, 3300, 4200);
    if (v < 3300)
        ipacket.batteryLevel = 0;
    else if (v < 3600)
        ipacket.batteryLevel = 1;
    else if (v < 3760)
        ipacket.batteryLevel = 2;
    else if (v < 3900)
        ipacket.batteryLevel = 3;
    else
        ipacket.batteryLevel = 4;
    if (chara->getSubscribedCount())
        chara->notify((uint8_t *)&ipacket, len, true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
}
void SwitchProController::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    uint8_t id = getRptID(pCharacteristic);
    // ESP_LOGI(TAG, "Report 0x%02X: writting!", id);
    const NimBLEAttValue &attv = pCharacteristic->getValue(nullptr);
    const uint8_t *data = attv.data();
    // uint16_t len = attv.length();
    // for (size_t i = 0; i < len; i++)
    // {
    //     printf("%02X ", data[i]);
    // }
    // printf("\r\n");

    switch (id)
    {
    case HID_RPT_ID_OUTPUT_RUMBLE_SUBCMD:
        handleOutputRptRumbleSubCMD((Output_Rumble_SubCMD *)data);
        break;
    case HID_RPT_ID_OUTPUT_RUMBLE:
        handleOutputRptRumble((Output_Rumble *)data);
        break;
    default:
        ESP_LOGI(TAG, "!!!!unhandle output report: 0x%02x!!!!", id);
        break;
    }
}

/** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
void SwitchProController::onNotify(NimBLECharacteristic *pCharacteristic)
{
    // uint8_t id = getRptID(pCharacteristic);
    // ESP_LOGI(TAG, "Report 0x%02X: send notify!", id);
}

/**
     *  The value returned in code is the NimBLE host return code.
     */
void SwitchProController::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
    // uint8_t id = getRptID(pCharacteristic);
    // ESP_LOGI(TAG, "Report 0x%02X: Notification/Indication return code: %d, %s",
    //          id, code, NimBLEUtils::returnCodeToString(code));
}

void SwitchProController::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
{

    std::string str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString();
    if (subValue == 0)
    {
        str += " Unsubscribed to ";
    }
    else if (subValue == 1)
    {
        str += " Subscribed to notfications for ";
    }
    else if (subValue == 2)
    {
        str += " Subscribed to indications for ";
    }
    else if (subValue == 3)
    {
        str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID());

    uint8_t id = getRptID(pCharacteristic);
    ESP_LOGI(TAG, "Report 0x%02X: %s", id, str.c_str());
}