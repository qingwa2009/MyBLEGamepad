/**
 * 参考：
 * https://controllers.fandom.com/wiki/Sony_DualSense
 * SDL-SDL2\src\joystick\hidapi\SDL_hidapi_ps5.c
*/
/**
 * steam 好像不识别；
*/
#include "DualSenseController.h"
#include "NimBLEDevice.h"
#include "ble_uuid.h"
#include "helper.h"
#include "math.h"

#define VID_SIG 2 //(1=Bluetooth SIG, 2=USB SIG)
#define VID 0x054C
#define PID 0x0CE6
#define PID_VER 0x0101

#define HID_RPT_ID_INPUT_01 0x01 //Get Controller State (simplified)
#define HID_RPT_ID_INPUT_31 0x31 //Get Controller State

#define HID_RPT_ID_OUTPUT_31 0x31 //Set Controller State
#define HID_RPT_ID_OUTPUT_32 0x32
#define HID_RPT_ID_OUTPUT_33 0x33
#define HID_RPT_ID_OUTPUT_34 0x34
#define HID_RPT_ID_OUTPUT_35 0x35
#define HID_RPT_ID_OUTPUT_36 0x36
#define HID_RPT_ID_OUTPUT_37 0x37
#define HID_RPT_ID_OUTPUT_38 0x38
#define HID_RPT_ID_OUTPUT_39 0x39

#define HID_RPT_ID_FEAT_05 0x05 //Get Calibration
#define HID_RPT_ID_FEAT_08 0x08
#define HID_RPT_ID_FEAT_09 0x09
#define HID_RPT_ID_FEAT_20 0x20
#define HID_RPT_ID_FEAT_22 0x22
#define HID_RPT_ID_FEAT_80 0x80
#define HID_RPT_ID_FEAT_81 0x81
#define HID_RPT_ID_FEAT_82 0x82
#define HID_RPT_ID_FEAT_83 0x83
#define HID_RPT_ID_FEAT_F0 0xF0
#define HID_RPT_ID_FEAT_F1 0xF1
#define HID_RPT_ID_FEAT_F2 0xF2

#define DELAY(x) vTaskDelay((x) / portTICK_PERIOD_MS)

static const char *TAG = "dualsense";

static const uint8_t reportmap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x35,       //   Usage (Rz)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x39,       //   Usage (Hat switch)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x07,       //   Logical Maximum (7)
    0x35, 0x00,       //   Physical Minimum (0)
    0x46, 0x3B, 0x01, //   Physical Maximum (315)
    0x65, 0x14,       //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x42,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x65, 0x00,       //   Unit (None)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (0x01)
    0x29, 0x0E,       //   Usage Maximum (0x0E)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x0E,       //   Report Count (14)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x01,       //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x4D,       //   Report Count (77)
    0x85, 0x31,       //   Report ID (49)
    0x09, 0x31,       //   Usage (0x31)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x09, 0x3B,       //   Usage (0x3B)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x85, 0x32,       //   Report ID (50)
    0x09, 0x32,       //   Usage (0x32)
    0x95, 0x8D,       //   Report Count (141)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x33,       //   Report ID (51)
    0x09, 0x33,       //   Usage (0x33)
    0x95, 0xCD,       //   Report Count (205)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x34,       //   Report ID (52)
    0x09, 0x34,       //   Usage (0x34)
    0x96, 0x0D, 0x01, //   Report Count (269)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x35,       //   Report ID (53)
    0x09, 0x35,       //   Usage (0x35)
    0x96, 0x4D, 0x01, //   Report Count (333)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x36,       //   Report ID (54)
    0x09, 0x36,       //   Usage (0x36)
    0x96, 0x8D, 0x01, //   Report Count (397)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x37,       //   Report ID (55)
    0x09, 0x37,       //   Usage (0x37)
    0x96, 0xCD, 0x01, //   Report Count (461)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x38,       //   Report ID (56)
    0x09, 0x38,       //   Usage (0x38)
    0x96, 0x0D, 0x02, //   Report Count (525)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x39,       //   Report ID (57)
    0x09, 0x39,       //   Usage (0x39)
    0x96, 0x22, 0x02, //   Report Count (546)
    0x91, 0x02,       //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x06, 0x80, 0xFF, //   Usage Page (Vendor Defined 0xFF80)
    0x85, 0x05,       //   Report ID (5)
    0x09, 0x33,       //   Usage (0x33)
    0x95, 0x28,       //   Report Count (40)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x08,       //   Report ID (8)
    0x09, 0x34,       //   Usage (0x34)
    0x95, 0x2F,       //   Report Count (47)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x09,       //   Report ID (9)
    0x09, 0x24,       //   Usage (0x24)
    0x95, 0x13,       //   Report Count (19)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x20,       //   Report ID (32)
    0x09, 0x26,       //   Usage (0x26)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x22,       //   Report ID (34)
    0x09, 0x40,       //   Usage (0x40)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x80,       //   Report ID (128)
    0x09, 0x28,       //   Usage (0x28)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x81,       //   Report ID (129)
    0x09, 0x29,       //   Usage (0x29)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x82,       //   Report ID (130)
    0x09, 0x2A,       //   Usage (0x2A)
    0x95, 0x09,       //   Report Count (9)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0x83,       //   Report ID (131)
    0x09, 0x2B,       //   Usage (0x2B)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF1,       //   Report ID (241)
    0x09, 0x31,       //   Usage (0x31)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF2,       //   Report ID (242)
    0x09, 0x32,       //   Usage (0x32)
    0x95, 0x0F,       //   Report Count (15)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF0,       //   Report ID (240)
    0x09, 0x30,       //   Usage (0x30)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,             // End Collection

    // 279 bytes
};

DualSenseController::DualSenseController(const std::string &deviceName) : IMyGamepad(deviceName)
{
    ipRptMode = HID_RPT_ID_INPUT_01;

    //改BLE地址，以区分不同手柄
    int isNRpa;
    ble_hs_id_copy_addr(0, mac, &isNRpa);
    ESP_LOGI(TAG, "BLE Public Addr: %02x:%02x:%02x:%02x:%02x:%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

    pHidDev->pnp(VID_SIG, BUILD_UINT16(HI_UINT16(VID), LO_UINT16(VID)), BUILD_UINT16(HI_UINT16(PID), LO_UINT16(PID)), BUILD_UINT16(HI_UINT16(PID_VER), LO_UINT16(PID_VER)));
    pHidDev->hidInfo(0x00, 0x01);
    pHidDev->reportMap((uint8_t *)reportmap, sizeof(reportmap));

    _iRptChara01 = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_01);
    _iRptChara01->setCallbacks(this);
    _iRptChara31 = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT_31);
    _iRptChara31->setCallbacks(this);

    _oRptChara31 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_31);
    _oRptChara31->setCallbacks(this);
    _oRptChara32 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_32);
    _oRptChara32->setCallbacks(this);
    _oRptChara33 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_33);
    _oRptChara33->setCallbacks(this);
    _oRptChara34 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_34);
    _oRptChara34->setCallbacks(this);
    _oRptChara35 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_35);
    _oRptChara35->setCallbacks(this);
    _oRptChara36 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_36);
    _oRptChara36->setCallbacks(this);
    _oRptChara37 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_37);
    _oRptChara37->setCallbacks(this);
    _oRptChara38 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_38);
    _oRptChara38->setCallbacks(this);
    _oRptChara39 = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_39);
    _oRptChara39->setCallbacks(this);

    _fRptChara05 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_05);
    _fRptChara05->setCallbacks(this);
    _fRptChara08 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_08);
    _fRptChara08->setCallbacks(this);
    _fRptChara09 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_09);
    _fRptChara09->setCallbacks(this);
    _fRptChara20 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_20);
    _fRptChara20->setCallbacks(this);
    _fRptChara22 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_22);
    _fRptChara22->setCallbacks(this);
    _fRptChara80 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_80);
    _fRptChara80->setCallbacks(this);
    _fRptChara81 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_81);
    _fRptChara81->setCallbacks(this);
    _fRptChara82 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_82);
    _fRptChara82->setCallbacks(this);
    _fRptChara83 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_83);
    _fRptChara83->setCallbacks(this);
    _fRptCharaF0 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_F0);
    _fRptCharaF0->setCallbacks(this);
    _fRptCharaF1 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_F1);
    _fRptCharaF1->setCallbacks(this);
    _fRptCharaF2 = pHidDev->featureReport((uint8_t)HID_RPT_ID_FEAT_F2);
    _fRptCharaF2->setCallbacks(this);

    _initStaticCharaValues();
    _enableIMU(true);
}
DualSenseController::~DualSenseController() {}

void DualSenseController::_initStaticCharaValues()
{
    //Get Calibration 全0
    struct ReportFeature05 f05 = {};

    _fRptChara20->setValue((uint8_t *)&f05, sizeof(struct ReportFeature05));

    struct ReportFeature09 f09 = {};
    //Get Controller and Host MAC，顺序不知对不对
    f09.ClientMac[0] = mac[5];
    f09.ClientMac[1] = mac[4];
    f09.ClientMac[2] = mac[3];
    f09.ClientMac[3] = mac[2];
    f09.ClientMac[4] = mac[1];
    f09.ClientMac[5] = mac[0];
    _fRptChara09->setValue((uint8_t *)&f09, sizeof(struct ReportFeature09));

    //Get Controller Version/Date (Firmware Info)，
    //参考SDL-SDL2 joystick\hidapi\SDL_hidapi_ps5.c
    struct ReportFeature20 f20 = {};
    f20.HardwareInfo = 0x0000FF00;
    // f20.UpdateVersion = 0x0220;
    f20.UpdateVersion = 0x0458;

    _fRptChara20->setValue((uint8_t *)&f20, sizeof(struct ReportFeature20));
}

bool DualSenseController::isRumbling()
{
    uint16_t l, r;
    getMotorLevel(&l, &r);
    return l > 0 || r > 0;
}

void DualSenseController::_handleReportIn01(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    static uint8_t counter = 0;
    uint8_t dir;
    struct ReportIn01 ip01 = {};

    ip01.State.ButtonCross = keys->A;
    ip01.State.ButtonCircle = keys->B;
    ip01.State.ButtonSquare = keys->X;
    ip01.State.ButtonTriangle = keys->Y;
    ip01.State.ButtonL1 = keys->LB;
    ip01.State.ButtonL2 = lt > 0;
    ip01.State.ButtonL3 = keys->LS;
    ip01.State.ButtonR1 = keys->RB;
    ip01.State.ButtonR2 = rt > 0;
    ip01.State.ButtonR3 = keys->RS;
    ip01.State.ButtonOptions = keys->Start;
    ip01.State.ButtonHome = btnXbox;
    ip01.State.ButtonShare = btnSelect;
    ip01.State.ButtonPad = 0;
    ip01.State.TriggerLeft = (uint8_t)((lt * 255L) / 100);
    ip01.State.TriggerRight = (uint8_t)((rt * 255L) / 100);

    ip01.State.LeftStickX = (uint8_t)((x * 255L) / 100);
    ip01.State.LeftStickY = (uint8_t)((y * 255L) / 100);
    ip01.State.RightStickX = (uint8_t)((z * 255L) / 100);
    ip01.State.RightStickY = (uint8_t)((rz * 255L) / 100);

    counter++;
    if (counter > 63)
        counter = 0;
    ip01.State.Counter = counter;

    dir = this->dirKey2DPadValue(keys->Up, keys->Right, keys->Down, keys->Left);
    dir = dir == 0 ? 8 : dir - 1;
    ip01.State.DPad = (enum Direction)dir;

    if (_iRptChara01->getSubscribedCount())
    {
        _iRptChara01->notify((uint8_t *)&ip01, sizeof(ip01), true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
    }
}

extern "C" unsigned int crc32_le(unsigned int crc, unsigned char const *buf, unsigned int len);

void DualSenseController::_handleReportIn31(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    struct ReportIn31 ip31 = {};
    int16_t motiondata[6];
    uint8_t dir;

    ip31.Data.HasHID = 1;
    ip31.Data.State.StateData.SeqNo = 0x01;
    ip31.Data.State.StateData.ButtonCross = keys->A;
    ip31.Data.State.StateData.ButtonCircle = keys->B;
    ip31.Data.State.StateData.ButtonSquare = keys->X;
    ip31.Data.State.StateData.ButtonTriangle = keys->Y;
    ip31.Data.State.StateData.ButtonL1 = keys->LB;
    ip31.Data.State.StateData.ButtonL2 = lt > 0;
    ip31.Data.State.StateData.ButtonL3 = keys->LS;
    ip31.Data.State.StateData.ButtonR1 = keys->RB;
    ip31.Data.State.StateData.ButtonR2 = rt > 0;
    ip31.Data.State.StateData.ButtonR3 = keys->RS;
    ip31.Data.State.StateData.ButtonOptions = keys->Start;
    ip31.Data.State.StateData.ButtonHome = btnXbox;
    ip31.Data.State.StateData.ButtonCreate = btnSelect;
    ip31.Data.State.StateData.ButtonPad = 0;
    ip31.Data.State.StateData.TriggerLeft = (uint8_t)((lt * 255L) / 100);
    ip31.Data.State.StateData.TriggerRight = (uint8_t)((rt * 255L) / 100);

    ip31.Data.State.StateData.LeftStickX = (uint8_t)((x * 255L) / 100);
    ip31.Data.State.StateData.LeftStickY = (uint8_t)((y * 255L) / 100);
    ip31.Data.State.StateData.RightStickX = (uint8_t)((z * 255L) / 100);
    ip31.Data.State.StateData.RightStickY = (uint8_t)((rz * 255L) / 100);

    dir = this->dirKey2DPadValue(keys->Up, keys->Right, keys->Down, keys->Left);
    dir = dir == 0 ? 8 : dir - 1;
    ip31.Data.State.StateData.DPad = (enum Direction)dir;

    getIMUMotionData(motiondata);

    ip31.Data.State.StateData.AccelerometerX = -motiondata[1];
    ip31.Data.State.StateData.AccelerometerY = motiondata[0];
    ip31.Data.State.StateData.AccelerometerZ = motiondata[2];
    ip31.Data.State.StateData.AngularVelocityX = -motiondata[4];
    ip31.Data.State.StateData.AngularVelocityY = motiondata[3];
    ip31.Data.State.StateData.AngularVelocityZ = motiondata[5];

    ip31.Data.State.StateData.SensorTimestamp = (uint32_t)esp_timer_get_time();

    ip31.Data.State.StateData.PowerPercent = (getBatteryLevel() / 10);
    ip31.Data.State.StateData.PowerState = isBatteryCharging() ? PowerState_Charging : (isBatteryChargeFull() ? PowerState_Complete : PowerState_Discharging);

    ip31.Data.State.StateData.HostTimestamp = hostTimeStamp;

    uint8_t *data = (uint8_t *)&ip31;
    uint16_t hdr = 0x31A1;
    uint32_t crc;
    crc = crc32_le(0, (uint8_t *)&hdr, sizeof(hdr));
    crc = crc32_le(crc, data, sizeof(ip31) - sizeof(crc));

    ip31.CRC.CRC = crc;

    if (_iRptChara31->getSubscribedCount())
    {
        _iRptChara31->notify((uint8_t *)&ip31, sizeof(ip31), true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
    }
}

void DualSenseController::update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    // static bool hasOut31Ev = false;
    switch (ipRptMode)
    {
    case HID_RPT_ID_INPUT_01:
        if (!this->isKeyStateChange(keys, btnXbox, btnSelect, x, y, z, rz, lt, rt))
        {
            DELAY(SEND_PERIOD_MS);
            return;
        }
        _handleReportIn01(dt, keys, btnXbox, btnSelect, x, y, z, rz, lt, rt);
        // ESP_LOGI(TAG, "HID_RPT_ID_INPUT_01");
        DELAY(SEND_PERIOD_MS);
        break;
    case HID_RPT_ID_INPUT_31:
        // if (hasOut31Ev)
        _handleReportIn31(dt, keys, btnXbox, btnSelect, x, y, z, rz, lt, rt);
        // ESP_LOGI(TAG, "HID_RPT_ID_INPUT_31");
        DELAY(SEND_PERIOD_MS);
        break;
    default:
        DELAY(SEND_PERIOD_MS * 3);
        break;
    }
}

void DualSenseController::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    uint8_t id = getRptID(pCharacteristic);
    ESP_LOGI(TAG, "onRead id %02x", id);
    switch (id)
    {
    case HID_RPT_ID_FEAT_05: //Get Calibration
                             //静态值
        ipRptMode = HID_RPT_ID_INPUT_31;
        break;
    case HID_RPT_ID_FEAT_09: //Get Controller and Host MAC
                             //静态值
        break;
    case HID_RPT_ID_FEAT_20: //Get Controller Version/Date (Firmware Info)
                             //静态值
        break;
    default:
        ESP_LOGI(TAG, "Unhandle Report 0x%02X: reading!!!", id);
        break;
    }
}
void DualSenseController::_enableIMU(bool enable)
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

void DualSenseController::_handleReportOut31(struct ReportOut31 *rpt)
{
    // ipRptMode = rpt->Data.EnableHID ? HID_RPT_ID_INPUT_01 : HID_RPT_ID_INPUT_31;
    hostTimeStamp = rpt->Data.State.HostTimestamp;
    if (rpt->Data.State.EnableImprovedRumbleEmulation)
        setMotorLevel((rpt->Data.State.RumbleEmulationLeft * 100) / 255, (rpt->Data.State.RumbleEmulationRight * 100) / 255);
    else
        setMotorLevel((rpt->Data.State.RumbleEmulationLeft * 100) / 127, (rpt->Data.State.RumbleEmulationRight * 100) / 127);
    // ESP_LOGI(TAG, "motor %d %d", rpt->Data.State.RumbleEmulationLeft, rpt->Data.State.RumbleEmulationRight);
}
void DualSenseController::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
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
    case HID_RPT_ID_OUTPUT_31:
        _handleReportOut31((struct ReportOut31 *)data);
        break;
    default:
        ESP_LOGI(TAG, "!!!!unhandle output report: 0x%02x!!!!", id);
        break;
    }
}

/** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
void DualSenseController::onNotify(NimBLECharacteristic *pCharacteristic)
{
    // uint8_t id = getRptID(pCharacteristic);
    // ESP_LOGI(TAG, "Report 0x%02X: send notify!", id);
}

/**
     *  The value returned in code is the NimBLE host return code.
     */
void DualSenseController::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
    // uint8_t id = getRptID(pCharacteristic);
    // ESP_LOGI(TAG, "Report 0x%02X: Notification/Indication return code: %d, %s",
    //          id, code, NimBLEUtils::returnCodeToString(code));
}

void DualSenseController::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
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