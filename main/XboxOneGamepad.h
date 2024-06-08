#ifndef _XBOX_ONE_GAMEPAD_H_
#define _XBOX_ONE_GAMEPAD_H_

#include "IMyGamepad.h"

#pragma pack(push, 1)
typedef struct
{
    unsigned short x;           // Usage 0x00010030: X, Value = 0 to 65535
    unsigned short y;           // Usage 0x00010031: Y, Value = 0 to 65535
    unsigned short z;           // Usage 0x00010032: Z, Value = 0 to 65535
    unsigned short Rz;          // Usage 0x00010035: Rz, Value = 0 to 65535
    unsigned short LT : 10;     // Usage 0x000200C5: Brake, Value = 0 to 1023
    unsigned short : 6;         // Pad
    unsigned short RT : 10;     // Usage 0x000200C4: Accelerator, Value = 0 to 1023
    unsigned short : 6;         // Pad
    unsigned char hat : 4;      // Usage 0x00010039: Hat switch, Value = 1 to 8, Physical = (Value - 1) x 45 in degrees
    unsigned char : 4;          // Pad
    unsigned char btnA : 1;     // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
    unsigned char btnB : 1;     // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
    unsigned char btn3 : 1;     // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
    unsigned char btnX : 1;     // Usage 0x00090004: Button 4, Value = 0 to 1
    unsigned char btnY : 1;     // Usage 0x00090005: Button 5, Value = 0 to 1
    unsigned char btn6 : 1;     // Usage 0x00090006: Button 6, Value = 0 to 1
    unsigned char btnL : 1;     // Usage 0x00090007: Button 7, Value = 0 to 1
    unsigned char btnR : 1;     // Usage 0x00090008: Button 8, Value = 0 to 1
    unsigned char btn9 : 1;     // Usage 0x00090009: Button 9, Value = 0 to 1
    unsigned char btn10 : 1;    // Usage 0x0009000A: Button 10, Value = 0 to 1
    unsigned char btnBack : 1;  // Usage 0x0009000B: Button 11, Value = 0 to 1
    unsigned char btnStart : 1; // Usage 0x0009000C: Button 12, Value = 0 to 1
    unsigned char btnXbox : 1;  // Usage 0x0009000D: Button 13, Value = 0 to 1
    unsigned char btnLS : 1;    // Usage 0x0009000E: Button 14, Value = 0 to 1
    unsigned char btnRS : 1;    // Usage 0x0009000F: Button 15, Value = 0 to 1
    unsigned char : 1;          // Pad
    unsigned char Record : 1;   //
    unsigned char : 7;          // Pad
} Gamepad_Input_Xbox;
typedef struct
{
    unsigned char enableAcutator : 4; // Usage 0x000F0097: DC Enable Actuators, Value = 0 to 1
    unsigned char : 4;                // Pad
    unsigned char lTMagnitude;        // Usage 0x000F0070: Magnitude, Value = 0 to 100
    unsigned char rTMagnitude;        // Usage 0x000F0070: Magnitude, Value = 0 to 100
    unsigned char strongMagnitude;    // Usage 0x000F0070: Magnitude, Value = 0 to 100
    unsigned char weakMagnitude;      // Usage 0x000F0070: Magnitude, Value = 0 to 100
    unsigned char duration;           // Usage 0x000F0050: Duration, Value = 0 to 255, Physical = Value in 10⁻² s units
    unsigned char startDelay;         // Usage 0x000F00A7: Start Delay, Value = 0 to 255, Physical = Value in 10⁻² s units
    unsigned char loopCount;          // Usage 0x000F007C: Loop Count, Value = 0 to 255
} OutputVibration;
#pragma pack(pop)

class XboxOneGamepad : public IMyGamepad, public NimBLECharacteristicCallbacks
{
private:
    NimBLECharacteristic *_iRptChara;
    NimBLECharacteristic *_oRptChara;
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onNotify(NimBLECharacteristic *pCharacteristic) override;
    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override;

public:
    XboxOneGamepad(const std::string &deviceName);
    ~XboxOneGamepad();
    void update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt) override;
};

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
}
#endif

#endif