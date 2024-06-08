/**
 * 参考：
 * https://github.com/Mystfit/ESP32-BLE-CompositeHID.git
*/
#include "XboxOneGamepad.h"
#include "ble_uuid.h"

#define HID_RPT_ID_INPUT 0x01 // input report ID
#define HID_RPT_ID_OUTPUT_VIBRATION 0x03

#define VID_SIG 2 //(1=Bluetooth SIG, 2=USB SIG)
#define VID 0x045E
#define PID 0x0B13
#define PID_VER 0x0509

#define DELAY(x) vTaskDelay((x) / portTICK_PERIOD_MS)

static const char *TAG = "xbox";

static const uint8_t reportmap[] = {
    0x05, 0x01,                        //(GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page
    0x09, 0x05,                        //(LOCAL)  USAGE              0x00010005 Game Pad (Application Collection)
    0xA1, 0x01,                        //(MAIN)   COLLECTION         0x01 Application (Usage=0x00010005: Page=Generic Desktop Page, Usage=Game Pad, Type=Application Collection)
    0x85, HID_RPT_ID_INPUT,            //  (GLOBAL) REPORT_ID          0x01 (1)
    0x09, 0x01,                        //  (LOCAL)  USAGE              0x00010001 Pointer (Physical Collection)
    0xA1, 0x00,                        //  (MAIN)   COLLECTION         0x00 Physical (Usage=0x00010001: Page=Generic Desktop Page, Usage=Pointer, Type=Physical Collection)
    0x09, 0x30,                        //    (LOCAL)  USAGE              0x00010030 X (Dynamic Value)
    0x09, 0x31,                        //    (LOCAL)  USAGE              0x00010031 Y (Dynamic Value)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
    0x27, 0xFF, 0xFF, 0x00, 0x00,      //    (GLOBAL) LOGICAL_MAXIMUM    0x0000FFFF (65535)
    0x95, 0x02,                        //    (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields
    0x75, 0x10,                        //    (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field
    0x81, 0x02,                        //    (MAIN)   INPUT              0x00000002 (2 fields x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0xC0,                              //  (MAIN)   END_COLLECTION     Physical
    0x09, 0x01,                        //  (LOCAL)  USAGE              0x00010001 Pointer (Physical Collection)
    0xA1, 0x00,                        //  (MAIN)   COLLECTION         0x00 Physical (Usage=0x00010001: Page=Generic Desktop Page, Usage=Pointer, Type=Physical Collection)
    0x09, 0x32,                        //    (LOCAL)  USAGE              0x00010032 Z (Dynamic Value)
    0x09, 0x35,                        //    (LOCAL)  USAGE              0x00010035 Rz (Dynamic Value)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x27, 0xFF, 0xFF, 0x00, 0x00,      //    (GLOBAL) LOGICAL_MAXIMUM    0x0000FFFF (65535) <-- Redundant: LOGICAL_MAXIMUM is already 65535
    0x95, 0x02,                        //    (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields <-- Redundant: REPORT_COUNT is already 2
    0x75, 0x10,                        //    (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field <-- Redundant: REPORT_SIZE is already 16
    0x81, 0x02,                        //    (MAIN)   INPUT              0x00000002 (2 fields x 16 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0xC0,                              //  (MAIN)   END_COLLECTION     Physical
    0x05, 0x02,                        //  (GLOBAL) USAGE_PAGE         0x0002 Simulation Controls Page
    0x09, 0xC5,                        //  (LOCAL)  USAGE              0x000200C5 Brake (Dynamic Value)
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x03,                  //  (GLOBAL) LOGICAL_MAXIMUM    0x03FF (1023)
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields
    0x75, 0x0A,                        //  (GLOBAL) REPORT_SIZE        0x0A (10) Number of bits per field
    0x81, 0x02,                        //  (MAIN)   INPUT              0x00000002 (1 field x 10 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x06,                        //  (GLOBAL) REPORT_SIZE        0x06 (6) Number of bits per field
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x81, 0x03,                        //  (MAIN)   INPUT              0x00000003 (1 field x 6 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x05, 0x02,                        //  (GLOBAL) USAGE_PAGE         0x0002 Simulation Controls Page <-- Redundant: USAGE_PAGE is already 0x0002
    0x09, 0xC4,                        //  (LOCAL)  USAGE              0x000200C4 Accelerator (Dynamic Value)
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x03,                  //  (GLOBAL) LOGICAL_MAXIMUM    0x03FF (1023)
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x75, 0x0A,                        //  (GLOBAL) REPORT_SIZE        0x0A (10) Number of bits per field
    0x81, 0x02,                        //  (MAIN)   INPUT              0x00000002 (1 field x 10 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x06,                        //  (GLOBAL) REPORT_SIZE        0x06 (6) Number of bits per field
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x81, 0x03,                        //  (MAIN)   INPUT              0x00000003 (1 field x 6 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x05, 0x01,                        //  (GLOBAL) USAGE_PAGE         0x0001 Generic Desktop Page
    0x09, 0x39,                        //  (LOCAL)  USAGE              0x00010039 Hat switch (Dynamic Value)
    0x15, 0x01,                        //  (GLOBAL) LOGICAL_MINIMUM    0x01 (1)
    0x25, 0x08,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x08 (8)
    0x35, 0x00,                        //  (GLOBAL) PHYSICAL_MINIMUM   0x00 (0)  <-- Info: Consider replacing 35 00 with 34
    0x46, 0x3B, 0x01,                  //  (GLOBAL) PHYSICAL_MAXIMUM   0x013B (315)
    0x66, 0x14, 0x00,                  //  (GLOBAL) UNIT               0x0014 Rotation in degrees [1° units] (4=System=English Rotation, 1=Rotation=Degrees)  <-- Info: Consider replacing 66 1400 with 65 14
    0x75, 0x04,                        //  (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x81, 0x42,                        //  (MAIN)   INPUT              0x00000042 (1 field x 4 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 1=Null 0=NonVolatile 0=Bitmap
    0x75, 0x04,                        //  (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field <-- Redundant: REPORT_SIZE is already 4
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x35, 0x00,                        //  (GLOBAL) PHYSICAL_MINIMUM   0x00 (0) <-- Redundant: PHYSICAL_MINIMUM is already 0 <-- Info: Consider replacing 35 00 with 34
    0x45, 0x00,                        //  (GLOBAL) PHYSICAL_MAXIMUM   0x00 (0)  <-- Info: Consider replacing 45 00 with 44
    0x65, 0x00,                        //  (GLOBAL) UNIT               0x00 No unit (0=None)  <-- Info: Consider replacing 65 00 with 64
    0x81, 0x03,                        //  (MAIN)   INPUT              0x00000003 (1 field x 4 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x05, 0x09,                        //  (GLOBAL) USAGE_PAGE         0x0009 Button Page
    0x19, 0x01,                        //  (LOCAL)  USAGE_MINIMUM      0x00090001 Button 1 Primary/trigger (Selector, On/Off Control, Momentary Control, or One Shot Control)
    0x29, 0x0F,                        //  (LOCAL)  USAGE_MAXIMUM      0x0009000F Button 15 (Selector, On/Off Control, Momentary Control, or One Shot Control)
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,                        //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field
    0x95, 0x0F,                        //  (GLOBAL) REPORT_COUNT       0x0F (15) Number of fields
    0x81, 0x02,                        //  (MAIN)   INPUT              0x00000002 (15 fields x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x01,                        //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field <-- Redundant: REPORT_SIZE is already 1
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields
    0x81, 0x03,                        //  (MAIN)   INPUT              0x00000003 (1 field x 1 bit) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x05, 0x0C,                        //  (GLOBAL) USAGE_PAGE         0x000C Consumer Device Page
    0x0A, 0xB2, 0x00,                  //  (LOCAL)  USAGE              0x000C00B2 Record (On/Off Control)  <-- Info: Consider replacing 0A B200 with 09 B2
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x75, 0x01,                        //  (GLOBAL) REPORT_SIZE        0x01 (1) Number of bits per field <-- Redundant: REPORT_SIZE is already 1
    0x81, 0x02,                        //  (MAIN)   INPUT              0x00000002 (1 field x 1 bit) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x15, 0x00,                        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //  (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x07,                        //  (GLOBAL) REPORT_SIZE        0x07 (7) Number of bits per field
    0x95, 0x01,                        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x81, 0x03,                        //  (MAIN)   INPUT              0x00000003 (1 field x 7 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x05, 0x0F,                        //  (GLOBAL) USAGE_PAGE         0x000F Physical Interface Device Page
    0x09, 0x21,                        //  (LOCAL)  USAGE              0x000F0021 Set Effect Report (Logical Collection)
    0x85, HID_RPT_ID_OUTPUT_VIBRATION, //  (GLOBAL) REPORT_ID          0x03 (3)
    0xA1, 0x02,                        //  (MAIN)   COLLECTION         0x02 Logical (Usage=0x000F0021: Page=Physical Interface Device Page, Usage=Set Effect Report, Type=Logical Collection)
    0x09, 0x97,                        //    (LOCAL)  USAGE              0x000F0097 DC Enable Actuators (Selector)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x01,                        //    (GLOBAL) LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x04,                        //    (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field
    0x95, 0x01,                        //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x91, 0x02,                        //    (MAIN)   OUTPUT             0x00000002 (1 field x 4 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x00,                        //    (GLOBAL) LOGICAL_MAXIMUM    0x00 (0)  <-- Info: Consider replacing 25 00 with 24
    0x75, 0x04,                        //    (GLOBAL) REPORT_SIZE        0x04 (4) Number of bits per field <-- Redundant: REPORT_SIZE is already 4
    0x95, 0x01,                        //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x91, 0x03,                        //    (MAIN)   OUTPUT             0x00000003 (1 field x 4 bits) 1=Constant 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x09, 0x70,                        //    (LOCAL)  USAGE              0x000F0070 Magnitude (Dynamic Value)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x25, 0x64,                        //    (GLOBAL) LOGICAL_MAXIMUM    0x64 (100)
    0x75, 0x08,                        //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x04,                        //    (GLOBAL) REPORT_COUNT       0x04 (4) Number of fields
    0x91, 0x02,                        //    (MAIN)   OUTPUT             0x00000002 (4 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x09, 0x50,                        //    (LOCAL)  USAGE              0x000F0050 Duration (Dynamic Value)
    0x66, 0x01, 0x10,                  //    (GLOBAL) UNIT               0x1001 Time in seconds [1 s units] (1=System=SI Linear, 1=Time=Seconds)
    0x55, 0x0E,                        //    (GLOBAL) UNIT_EXPONENT      0x0E (Unit Value x 10⁻²)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,                  //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)
    0x75, 0x08,                        //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8
    0x95, 0x01,                        //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields
    0x91, 0x02,                        //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x09, 0xA7,                        //    (LOCAL)  USAGE              0x000F00A7 Start Delay (Dynamic Value)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,                  //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255) <-- Redundant: LOGICAL_MAXIMUM is already 255
    0x75, 0x08,                        //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8
    0x95, 0x01,                        //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x91, 0x02,                        //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x65, 0x00,                        //    (GLOBAL) UNIT               0x00 No unit (0=None)  <-- Info: Consider replacing 65 00 with 64
    0x55, 0x00,                        //    (GLOBAL) UNIT_EXPONENT      0x00 (Unit Value x 10⁰)  <-- Info: Consider replacing 55 00 with 54
    0x09, 0x7C,                        //    (LOCAL)  USAGE              0x000F007C Loop Count (Dynamic Value)
    0x15, 0x00,                        //    (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0 <-- Info: Consider replacing 15 00 with 14
    0x26, 0xFF, 0x00,                  //    (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255) <-- Redundant: LOGICAL_MAXIMUM is already 255
    0x75, 0x08,                        //    (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field <-- Redundant: REPORT_SIZE is already 8
    0x95, 0x01,                        //    (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields <-- Redundant: REPORT_COUNT is already 1
    0x91, 0x02,                        //    (MAIN)   OUTPUT             0x00000002 (1 field x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0xC0,                              //(MAIN)   END_COLLECTION     Logical
    0xC0,                              //(MAIN)   END_COLLECTION     Application

};

XboxOneGamepad::XboxOneGamepad(const std::string &deviceName) : IMyGamepad(deviceName)
{

    pHidDev->pnp(VID_SIG, BUILD_UINT16(HI_UINT16(VID), LO_UINT16(VID)), BUILD_UINT16(HI_UINT16(PID), LO_UINT16(PID)), BUILD_UINT16(HI_UINT16(PID_VER), LO_UINT16(PID_VER)));
    pHidDev->hidInfo(0x00, 0x01);
    pHidDev->reportMap((uint8_t *)reportmap, sizeof(reportmap));

    _iRptChara = pHidDev->inputReport((uint8_t)HID_RPT_ID_INPUT);
    _oRptChara = pHidDev->outputReport((uint8_t)HID_RPT_ID_OUTPUT_VIBRATION);
    _oRptChara->setCallbacks(this);
}
XboxOneGamepad::~XboxOneGamepad() {}

void XboxOneGamepad::update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    if (!this->isKeyStateChange(keys, btnXbox, btnSelect, x, y, z, rz, lt, rt))
    {
        DELAY(SEND_PERIOD_MS);
        return;
    }

    static Gamepad_Input_Xbox ip = {};
    ip.btnA = keys->A;
    ip.btnB = keys->B;
    ip.btnX = keys->X;
    ip.btnY = keys->Y;
    ip.btnL = keys->LB;
    ip.btnR = keys->RB;
    ip.btnLS = keys->LS;
    ip.btnRS = keys->RS;
    ip.btnStart = keys->Start;
    ip.btnXbox = btnXbox;
    ip.btnBack = btnSelect;
    float ff;
    ff = (x * 65535L) / 100;
    ip.x = (uint16_t)ff;
    ff = (y * 65535L) / 100;
    ip.y = (uint16_t)ff;
    ff = (z * 65535L) / 100;
    ip.z = (uint16_t)ff;
    ff = (rz * 65535L) / 100;
    ip.Rz = (uint16_t)ff;
    ff = (lt * 1023L) / 100;
    ip.LT = (uint16_t)ff;
    ff = (rt * 1023L) / 100;
    ip.RT = (uint16_t)ff;

    ip.hat = this->dirKey2DPadValue(keys->Up, keys->Right, keys->Down, keys->Left);

    if (_iRptChara->getSubscribedCount())
    {
        // ESP_LOGI(TAG, "BIBI");
        _iRptChara->notify((uint8_t *)&ip, sizeof(ip), true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
    }
    DELAY(SEND_PERIOD_MS);
}

void XboxOneGamepad::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    ESP_LOGI(TAG, "%s : onRead(), value: %s\n",
             pCharacteristic->getUUID().toString().c_str(),
             pCharacteristic->getValue().c_str());
}

void XboxOneGamepad::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    if (pCharacteristic == _oRptChara)
    {

        OutputVibration op = pCharacteristic->getValue<OutputVibration>();
        // ESP_LOGI(TAG, "%s : onWrite(), value: %d %d %d %d %d %d %d %d\n",
        //          pCharacteristic->getUUID().toString().c_str(),
        //          op.enableAcutator, op.lTMagnitude, op.rTMagnitude,
        //          op.strongMagnitude, op.weakMagnitude, op.duration, op.startDelay, op.loopCount);

        setMotorLevel(op.strongMagnitude, op.weakMagnitude);
    }
}

/** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
void XboxOneGamepad::onNotify(NimBLECharacteristic *pCharacteristic)
{
    ESP_LOGI(TAG, "Sending notification to clients\n");
}

/**
     *  The value returned in code is the NimBLE host return code.
     */
void XboxOneGamepad::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
    ESP_LOGI(TAG, "Notification/Indication return code: %d, %s\n",
             code, NimBLEUtils::returnCodeToString(code));
}

void XboxOneGamepad::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
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

    ESP_LOGI(TAG, "%s\n", str.c_str());
}