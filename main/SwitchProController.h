#ifndef _SWITCH_PRO_CONTROLLER_H_
#define _SWITCH_PRO_CONTROLLER_H_
#include "IMyGamepad.h"
#include "stdint.h"
#include "assert.h"

#pragma pack(push, 1)
typedef struct
{
    uint8_t btnA : 1;       // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
    uint8_t btnB : 1;       // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
    uint8_t btnX : 1;       // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
    uint8_t btnY : 1;       // Usage 0x00090004: Button 4, Value = 0 to 1
    uint8_t btnL : 1;       // Usage 0x00090005: Button 5, Value = 0 to 1
    uint8_t btnR : 1;       // Usage 0x00090006: Button 6, Value = 0 to 1
    uint8_t btnZL : 1;      // Usage 0x00090007: Button 7, Value = 0 to 1
    uint8_t btnZR : 1;      // Usage 0x00090008: Button 8, Value = 0 to 1
    uint8_t btnMinus : 1;   // Usage 0x00090009: Button 9, Value = 0 to 1
    uint8_t btnPlus : 1;    // Usage 0x0009000A: Button 10, Value = 0 to 1
    uint8_t btnLS : 1;      // Usage 0x0009000B: Button 11, Value = 0 to 1
    uint8_t btnRS : 1;      // Usage 0x0009000C: Button 12, Value = 0 to 1
    uint8_t btnHome : 1;    // Usage 0x0009000D: Button 13, Value = 0 to 1
    uint8_t btnCapture : 1; // Usage 0x0009000E: Button 14, Value = 0 to 1
    uint8_t btn15 : 1;      // Usage 0x0009000F: Button 15, Value = 0 to 1
    uint8_t : 1;            // Pad
    uint8_t hat : 4;        // Usage 0x00010039: Hat switch, Value = 0 to 7, Physical = (Value - 1) x 45 in degrees
    uint8_t : 4;            // Pad

    uint16_t x;  // Usage 0x00010030: X, Value = 0 to 65535
    uint16_t y;  // Usage 0x00010031: Y, Value = 0 to 65535
    uint16_t z;  // Usage 0x00010032: Z, Value = 0 to 65535
    uint16_t Rz; // Usage 0x00010035: Rz, Value = 0 to 65535
} Gamepad_Input_Pro;
static_assert(sizeof(Gamepad_Input_Pro) == 11, "struct wrong size");

typedef struct
{
    int16_t accelx;
    int16_t accely;
    int16_t accelz;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
} IMU_Data;
static_assert(sizeof(IMU_Data) == 12, "struct wrong size");

typedef struct
{
    uint8_t timer;            //00~FF
    uint8_t connInfo : 4;     //0
    uint8_t charging : 1;     //0,1
    uint8_t batteryLevel : 3; //0,2,4,6,8
    union
    {
        uint8_t btns[3];
        struct
        {
            uint8_t Y : 1;
            uint8_t X : 1;
            uint8_t B : 1;
            uint8_t A : 1;
            uint8_t RSR : 1;
            uint8_t RSL : 1;
            uint8_t R : 1;
            uint8_t ZR : 1;

            uint8_t Minus : 1;
            uint8_t Plus : 1;
            uint8_t RS : 1;
            uint8_t LS : 1;
            uint8_t Home : 1;
            uint8_t Capture : 1;
            uint8_t : 1;
            uint8_t ChargingGrip : 1;

            uint8_t Down : 1;
            uint8_t Up : 1;
            uint8_t Right : 1;
            uint8_t Left : 1;
            uint8_t LSR : 1;
            uint8_t LSL : 1;
            uint8_t L : 1;
            uint8_t ZL : 1;
        } btn;
    };
    union
    {
        uint8_t sticks[6];
        struct
        {
            uint8_t left_H_LB_8 : 8;
            uint8_t left_H_HB_4 : 4;
            uint8_t left_V_LB_4 : 4;
            uint8_t left_V_HB_8 : 8;
            uint8_t right_H_LB_8 : 8;
            uint8_t right_H_HB_4 : 4;
            uint8_t right_V_LB_4 : 4;
            uint8_t right_V_HB_8 : 8;
        } stick;
    };
    uint8_t motorStatus; //
    union
    {
        struct
        {
            uint8_t subcmdReply; //msb 1 ACK,0 NACK, lsb 7 replay data type
            uint8_t subcmdReplyID;
            uint8_t data[34];
        };
        struct
        {
            IMU_Data imuData0;
            IMU_Data imuData1;
            IMU_Data imuData2;
        };
        uint8_t mcudata[36];
    };
    union
    {
        uint8_t nfc[313];
        uint8_t ir[313];
    };
} Input_Report;
static_assert(sizeof(Input_Report) == 361, "struct wrong size");

typedef struct
{
    uint8_t cmd;
    uint8_t mac[6];
    uint8_t fixed[3];
    uint8_t alias[20];
    uint8_t extra[8];
} SubCMD_01;
static_assert(sizeof(SubCMD_01) == 38, "struct wrong size");

typedef struct
{
    uint8_t rptMode; //
    uint8_t data[35];
    uint8_t crc;
    uint8_t tail;
} SubCMD_03;
static_assert(sizeof(SubCMD_03) == 38, "struct wrong size");

typedef struct
{
    uint16_t time;
} SubCMD_04;

typedef struct
{
    uint8_t hciMode;
} SubCMD_06;

typedef struct
{
    uint8_t enable;
} SubCMD_08;
typedef struct
{
    uint32_t address;
    uint8_t length;
} SubCMD_10;
typedef struct
{
    uint32_t address;
    uint8_t length;
    uint8_t data[29];
} SubCMD_11;
static_assert(sizeof(SubCMD_11) == 34, "struct wrong size");

typedef struct
{
    uint32_t address;
    uint8_t length;
} SubCMD_12;
typedef struct
{
    uint8_t state;
} SubCMD_22;
typedef struct
{
    uint8_t player : 4;
    uint8_t flashing : 4;
} SubCMD_30;
static_assert(sizeof(SubCMD_30) == 1, "struct wrong size");

typedef struct
{
    uint8_t base_duration : 4;   // 0_L : 1~F = 8~175ms, 0 = OFF
    uint8_t pattern_count : 4;   // 0_H :
    uint8_t repeat_count : 4;    // 1_L : 0 = forever
    uint8_t start_intensity : 4; //
    uint8_t patterns[23];
} SubCMD_38;
static_assert(sizeof(SubCMD_38) == 25, "struct wrong size");

typedef struct
{
    uint8_t enable;
} SubCMD_40;
typedef struct
{
    uint8_t gyroRange;
    uint8_t accelRange;
    uint8_t gyroSampleRate;
    uint8_t accelBandwidth;
} SubCMD_41;
typedef struct
{
    uint8_t address;
    uint8_t operation;
    uint8_t value;
} SubCMD_42;
typedef struct
{
    uint8_t address;
    uint8_t count;
} SubCMD_43;
typedef struct
{
    uint8_t enable;
} SubCMD_48;
typedef struct
{
    uint16_t voltage_mv;
} SubCMD_50;
typedef struct
{
    uint8_t freq_h;
    uint8_t freq_h_amp;
    uint8_t freq_l;
    uint8_t freq_l_amp;
} Rumble_Data;
typedef struct
{
    uint8_t packetNum; //00~0F
    struct
    {
        Rumble_Data lRumble;
        Rumble_Data rRumble;
    } rumble;
} Output_Rumble;
static_assert(sizeof(Output_Rumble) == 9, "struct wrong size");

typedef struct
{
    uint8_t packetNum; //00~0F
    struct
    {
        Rumble_Data lRumble;
        Rumble_Data rRumble;
    } rumble;
    uint8_t subcmd; //
    union
    {
        uint8_t raw[38];
        SubCMD_01 subcmd01;
        SubCMD_03 subcmd03;
        SubCMD_04 subcmd04;
        SubCMD_06 subcmd06;
        SubCMD_08 subcmd08;
        SubCMD_10 subcmd10;
        SubCMD_11 subcmd11;
        SubCMD_12 subcmd12;
        SubCMD_22 subcmd22;
        SubCMD_30 subcmd30;
        SubCMD_38 subcmd38;
        SubCMD_40 subcmd40;
        SubCMD_41 subcmd41;
        SubCMD_42 subcmd42;
        SubCMD_43 subcmd43;
        SubCMD_48 subcmd48;
    };
} Output_Rumble_SubCMD;
static_assert(sizeof(Output_Rumble_SubCMD) == 48, "struct wrong size");

#pragma pack(pop)

class SwitchProController : public IMyGamepad, public NimBLECharacteristicCallbacks
{
private:
    NimBLECharacteristic *_iRptChara;
    NimBLECharacteristic *_iRptChara_STD;
    NimBLECharacteristic *_iRptChara_Full;
    NimBLECharacteristic *_iRptChara_NFC_MCU;
    NimBLECharacteristic *_iRptChara_Unknown1;
    NimBLECharacteristic *_iRptChara_Unknown2;
    NimBLECharacteristic *_oRptChara_Rumble_SubCMD;
    NimBLECharacteristic *_oRptChara_Rumble;
    NimBLECharacteristic *_oRptChara_ReqNFC_Rumble;
    NimBLECharacteristic *_oRptChara_Unknown;
    uint8_t mac[6] = {};
    uint8_t packetTimer = 0;

    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onNotify(NimBLECharacteristic *pCharacteristic) override;
    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override;

    uint8_t ipRptMode;
    uint8_t isIMUEnable = 0;
    uint8_t isRumbleEnable = 0;

    void _enableIMU(bool enable);
    void enableVibration(bool enable);
    void setPlayerLED(const SubCMD_30 *data);
    void setLowPower(bool enable);
    void setHomeLED(const SubCMD_38 *data);
    void handleOutputRptRumble(const Output_Rumble *data);
    void handleOutputRptRumbleSubCMD(const Output_Rumble_SubCMD *subcmdPacket);
    void sendPacket(NimBLECharacteristic *chara, uint16_t len);
    void setSTDOrFullInputPacket(KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt);

public:
    SwitchProController(const std::string &deviceName);
    ~SwitchProController();
    void update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt) override;
    bool isRumbling();
};

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
}
#endif

#endif