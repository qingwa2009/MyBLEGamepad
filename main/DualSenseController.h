#ifndef _DUAL_SENSE_CONTROLLER_H_
#define _DUAL_SENSE_CONTROLLER_H_
#include "IMyGamepad.h"
#include "stdint.h"
#include "assert.h"
#include "freertos/event_groups.h"

#pragma pack(push, 1)

enum Direction : uint8_t
{
    Direction_North = 0,
    Direction_NorthEast,
    Direction_East,
    Direction_SouthEast,
    Direction_South,
    Direction_SouthWest,
    Direction_West,
    Direction_NorthWest,
    Direction_None = 8
};

enum PowerState : uint8_t
{
    PowerState_Discharging = 0x00,         // Use PowerPercent
    PowerState_Charging = 0x01,            // Use PowerPercent
    PowerState_Complete = 0x02,            // PowerPercent not valid? assume 100%?
    PowerState_AbnormalVoltage = 0x0A,     // PowerPercent not valid?
    PowerState_AbnormalTemperature = 0x0B, // PowerPercent not valid?
    PowerState_ChargingError = 0x0F        // PowerPercent not valid?
};

enum MuteLight : uint8_t
{
    MuteLight_Off = 0,
    MuteLight_On,
    MuteLight_Breathing,
    MuteLight_DoNothing, // literally nothing, this input is ignored,
                         // though it might be a faster blink in other versions
    MuteLight_NoAction4,
    MuteLight_NoAction5,
    MuteLight_NoAction6,
    MuteLight_NoAction7 = 7
};

enum LightBrightness : uint8_t
{
    LightBrightness_Bright = 0,
    LightBrightness_Mid,
    LightBrightness_Dim,
    LightBrightness_NoAction3,
    LightBrightness_NoAction4,
    LightBrightness_NoAction5,
    LightBrightness_NoAction6,
    LightBrightness_NoAction7 = 7
};

enum LightFadeAnimation : uint8_t
{
    LightFadeAnimation_Nothing = 0,
    LightFadeAnimation_FadeIn, // from black to blue
    LightFadeAnimation_FadeOut // from blue to black
};

template <int N>
struct BTCRC
{
    uint8_t Buff[N - 4];
    uint32_t CRC;
};

struct TouchFingerData
{ // 4
    /*0.0*/ uint32_t Index : 7;
    /*0.7*/ uint32_t NotTouching : 1;
    /*1.0*/ uint32_t FingerX : 12;
    /*2.4*/ uint32_t FingerY : 12;
};
static_assert(sizeof(TouchFingerData) == 4, "struct wrong size");

struct TouchData
{ // 9
    /*0*/ TouchFingerData Finger[2];
    /*8*/ uint8_t Timestamp;
};
static_assert(sizeof(TouchData) == 9, "struct wrong size");

struct BTSimpleGetStateData
{ // 9
    /*0  */ uint8_t LeftStickX;
    /*1  */ uint8_t LeftStickY;
    /*2  */ uint8_t RightStickX;
    /*3  */ uint8_t RightStickY;
    /*4.0*/ Direction DPad : 4;
    /*4.4*/ uint8_t ButtonSquare : 1;
    /*4.5*/ uint8_t ButtonCross : 1;
    /*4.6*/ uint8_t ButtonCircle : 1;
    /*4.7*/ uint8_t ButtonTriangle : 1;
    /*5.0*/ uint8_t ButtonL1 : 1;
    /*5.1*/ uint8_t ButtonR1 : 1;
    /*5.2*/ uint8_t ButtonL2 : 1;
    /*5.3*/ uint8_t ButtonR2 : 1;
    /*5.4*/ uint8_t ButtonShare : 1;
    /*5.5*/ uint8_t ButtonOptions : 1;
    /*5.6*/ uint8_t ButtonL3 : 1;
    /*5.7*/ uint8_t ButtonR3 : 1;
    /*6.1*/ uint8_t ButtonHome : 1;
    /*6.2*/ uint8_t ButtonPad : 1;
    /*6.3*/ uint8_t Counter : 6;
    /*7  */ uint8_t TriggerLeft;
    /*8  */ uint8_t TriggerRight;
    // anything beyond this point, if set, is invalid junk data that was not cleared
};
static_assert(sizeof(BTSimpleGetStateData) == 9, "struct wrong size");

struct USBGetStateData
{ // 63
    /* 0  */ uint8_t LeftStickX;
    /* 1  */ uint8_t LeftStickY;
    /* 2  */ uint8_t RightStickX;
    /* 3  */ uint8_t RightStickY;
    /* 4  */ uint8_t TriggerLeft;
    /* 5  */ uint8_t TriggerRight;
    /* 6  */ uint8_t SeqNo; // always 0x01 on BT
    /* 7.0*/ Direction DPad : 4;
    /* 7.4*/ uint8_t ButtonSquare : 1;
    /* 7.5*/ uint8_t ButtonCross : 1;
    /* 7.6*/ uint8_t ButtonCircle : 1;
    /* 7.7*/ uint8_t ButtonTriangle : 1;
    /* 8.0*/ uint8_t ButtonL1 : 1;
    /* 8.1*/ uint8_t ButtonR1 : 1;
    /* 8.2*/ uint8_t ButtonL2 : 1;
    /* 8.3*/ uint8_t ButtonR2 : 1;
    /* 8.4*/ uint8_t ButtonCreate : 1;
    /* 8.5*/ uint8_t ButtonOptions : 1;
    /* 8.6*/ uint8_t ButtonL3 : 1;
    /* 8.7*/ uint8_t ButtonR3 : 1;
    /* 9.0*/ uint8_t ButtonHome : 1;
    /* 9.1*/ uint8_t ButtonPad : 1;
    /* 9.2*/ uint8_t ButtonMute : 1;
    /* 9.3*/ uint8_t UNK1 : 1;                // appears unused
    /* 9.4*/ uint8_t ButtonLeftFunction : 1;  // DualSense Edge
    /* 9.5*/ uint8_t ButtonRightFunction : 1; // DualSense Edge
    /* 9.6*/ uint8_t ButtonLeftPaddle : 1;    // DualSense Edge
    /* 9.7*/ uint8_t ButtonRightPaddle : 1;   // DualSense Edge
    /*10  */ uint8_t UNK2;                    // appears unused
    /*11  */ uint32_t UNK_COUNTER;            // Linux driver calls this reserved, tools leak calls the 2 high bytes "random"
    /*15  */ int16_t AngularVelocityX;
    /*17  */ int16_t AngularVelocityZ;
    /*19  */ int16_t AngularVelocityY;
    /*21  */ int16_t AccelerometerX;
    /*23  */ int16_t AccelerometerY;
    /*25  */ int16_t AccelerometerZ;
    /*27  */ uint32_t SensorTimestamp;
    /*31  */ int8_t Temperature; // reserved2 in Linux driver
    /*32  */ struct TouchData TouchData;
    /*41.0*/ uint8_t TriggerRightStopLocation : 4; // trigger stop can be a range from 0 to 9 (F/9.0 for Apple interface)
    /*41.4*/ uint8_t TriggerRightStatus : 4;
    /*42.0*/ uint8_t TriggerLeftStopLocation : 4;
    /*42.4*/ uint8_t TriggerLeftStatus : 4;  // 0 feedbackNoLoad
                                             // 1 feedbackLoadApplied
                                             // 0 weaponReady
                                             // 1 weaponFiring
                                             // 2 weaponFired
                                             // 0 vibrationNotVibrating
                                             // 1 vibrationIsVibrating
    /*43  */ uint32_t HostTimestamp;         // mirrors data from report write
    /*47.0*/ uint8_t TriggerRightEffect : 4; // Active trigger effect, previously we thought this was status max
    /*47.4*/ uint8_t TriggerLeftEffect : 4;  // 0 for reset and all other effects
                                             // 1 for feedback effect
                                             // 2 for weapon effect
                                             // 3 for vibration
    /*48  */ uint32_t DeviceTimeStamp;
    /*52.0*/ uint8_t PowerPercent : 4; // 0x00-0x0A
    /*52.4*/ enum PowerState PowerState : 4;
    /*53.0*/ uint8_t PluggedHeadphones : 1;
    /*53.1*/ uint8_t PluggedMic : 1;
    /*53.2*/ uint8_t MicMuted : 1; // Mic muted by powersave/mute command
    /*53.3*/ uint8_t PluggedUsbData : 1;
    /*53.4*/ uint8_t PluggedUsbPower : 1;
    /*53.5*/ uint8_t PluggedUnk1 : 3;
    /*54.0*/ uint8_t PluggedExternalMic : 1;  // Is external mic active (automatic in mic auto mode)
    /*54.1*/ uint8_t HapticLowPassFilter : 1; // Is the Haptic Low-Pass-Filter active?
    /*54.2*/ uint8_t PluggedUnk3 : 6;
    /*55  */ uint8_t AesCmac[8];
};
static_assert(sizeof(USBGetStateData) == 63, "struct wrong size");

struct BTGetStateData
{
    /* 0*/ struct USBGetStateData StateData;
    /*63*/ uint8_t UNK1; // Oscillates between 00101100 and 00101101 when rumbling
                         // Not affected by rumble volume or enhanced vs normal rumble
                         // Audio rumble not yet tested as this is only on BT
    /*64*/ uint8_t BtCrcFailCount;
};
static_assert(sizeof(BTGetStateData) == 65, "struct wrong size");

struct SetStateData
{                                               // 47
    /*    */                                    // Report Set Flags
    /*    */                                    // These flags are used to indicate what contents from this report should be processed
    /* 0.0*/ uint8_t EnableRumbleEmulation : 1; // Suggest halving rumble strength
    /* 0.1*/ uint8_t UseRumbleNotHaptics : 1;   //
    /* 0.2*/ uint8_t AllowRightTriggerFFB : 1;  // Enable setting RightTriggerFFB
    /* 0.3*/ uint8_t AllowLeftTriggerFFB : 1;   // Enable setting LeftTriggerFFB
                                                /*    */
    /* 0.4*/ uint8_t AllowHeadphoneVolume : 1;  // Enable setting VolumeHeadphones
    /* 0.5*/ uint8_t AllowSpeakerVolume : 1;    // Enable setting VolumeSpeaker
    /* 0.6*/ uint8_t AllowMicVolume : 1;        // Enable setting VolumeMic
    /* 0.7*/ uint8_t AllowAudioControl : 1;     // Enable setting AudioControl section

    /* 1.0*/ uint8_t AllowMuteLight : 1;           // Enable setting MuteLightMode
    /* 1.1*/ uint8_t AllowAudioMute : 1;           // Enable setting MuteControl section
    /* 1.2*/ uint8_t AllowLedColor : 1;            // Enable RGB LED section                                                   /*    */
    /* 1.3*/ uint8_t ResetLights : 1;              // Release the LEDs from Wireless firmware control
    /*    */                                       // When in wireless mode this must be signaled to control LEDs
    /*    */                                       // This cannot be applied during the BT pair animation.
    /*    */                                       // SDL2 waits until the SensorTimestamp value is >= 10200000
    /*    */                                       // before pulsing this bit once.
                                                   /*    */
    /* 1.4*/ uint8_t AllowPlayerIndicators : 1;    // Enable setting PlayerIndicators section
    /* 1.5*/ uint8_t AllowHapticLowPassFilter : 1; // Enable HapticLowPassFilter
    /* 1.6*/ uint8_t AllowMotorPowerLevel : 1;     // MotorPowerLevel reductions for trigger/haptic
    /* 1.7*/ uint8_t AllowAudioControl2 : 1;       // Enable setting AudioControl2 section
                                                   /*    */
    /* 2  */ uint8_t RumbleEmulationRight;         // emulates the light weight
    /* 3  */ uint8_t RumbleEmulationLeft;          // emulated the heavy weight
                                                   /*    */
    /* 4  */ uint8_t VolumeHeadphones;             // max 0x7f
    /* 5  */ uint8_t VolumeSpeaker;                // PS5 appears to only use the range 0x3d-0x64
    /* 6  */ uint8_t VolumeMic;                    // not linier, seems to max at 64, 0 is not fully muted
                                                   /*    */
    /*    */                                       // AudioControl
    /* 7.0*/ uint8_t MicSelect : 2;                // 0 Auto
    /*    */                                       // 1 Internal Only
    /*    */                                       // 2 External Only
    /*    */                                       // 3 Unclear, sets external mic flag but might use internal mic, do test
    /* 7.2*/ uint8_t EchoCancelEnable : 1;
    /* 7.3*/ uint8_t NoiseCancelEnable : 1;
    /* 7.4*/ uint8_t OutputPathSelect : 2; // 0 L_R_X
    /*    */                               // 1 L_L_X
    /*    */                               // 2 L_L_R
    /*    */                               // 3 X_X_R
    /* 7.6*/ uint8_t InputPathSelect : 2;  // 0 CHAT_ASR
    /*    */                               // 1 CHAT_CHAT
    /*    */                               // 2 ASR_ASR
    /*    */                               // 3 Does Nothing, invalid
                                           /*    */
    /* 8  */ MuteLight MuteLightMode;
    /*    */
    /*    */ // MuteControl
    /* 9.0*/ uint8_t TouchPowerSave : 1;
    /* 9.1*/ uint8_t MotionPowerSave : 1;
    /* 9.2*/ uint8_t HapticPowerSave : 1; // AKA BulletPowerSave
    /* 9.3*/ uint8_t AudioPowerSave : 1;
    /* 9.4*/ uint8_t MicMute : 1;
    /* 9.5*/ uint8_t SpeakerMute : 1;
    /* 9.6*/ uint8_t HeadphoneMute : 1;
    /* 9.7*/ uint8_t HapticMute : 1; // AKA BulletMute
                                     /*    */
    /*10  */ uint8_t RightTriggerFFB[11];
    /*21  */ uint8_t LeftTriggerFFB[11];
    /*32  */ uint32_t HostTimestamp;                    // mirrored into report read
                                                        /*    */
    /*    */                                            // MotorPowerLevel
    /*36.0*/ uint8_t TriggerMotorPowerReduction : 4;    // 0x0-0x7 (no 0x8?) Applied in 12.5% reductions
    /*36.4*/ uint8_t RumbleMotorPowerReduction : 4;     // 0x0-0x7 (no 0x8?) Applied in 12.5% reductions
                                                        /*    */
    /*    */                                            // AudioControl2
    /*37.0*/ uint8_t SpeakerCompPreGain : 3;            // additional speaker volume boost
    /*37.3*/ uint8_t BeamformingEnable : 1;             // Probably for MIC given there's 2, might be more bits, can't find what it does
    /*37.4*/ uint8_t UnkAudioControl2 : 4;              // some of these bits might apply to the above
                                                        /*    */
    /*38.0*/ uint8_t AllowLightBrightnessChange : 1;    // LED_BRIHTNESS_CONTROL
    /*38.1*/ uint8_t AllowColorLightFadeAnimation : 1;  // LIGHTBAR_SETUP_CONTROL
    /*38.2*/ uint8_t EnableImprovedRumbleEmulation : 1; // Use instead of EnableRumbleEmulation
                                                        // requires FW >= 0x0224
                                                        // No need to halve rumble strength
    /*38.3*/ uint8_t UNKBITC : 5;                       // unused
                                                        /*    */
    /*39.0*/ uint8_t HapticLowPassFilter : 1;
    /*39.1*/ uint8_t UNKBIT : 7;
    /*    */
    /*40  */ uint8_t UNKBYTE; // previous notes suggested this was HLPF, was probably off by 1
                              /*    */
    /*41  */ enum LightFadeAnimation LightFadeAnimation;
    /*42  */ enum LightBrightness LightBrightness;
    /*    */
    /*    */                              // PlayerIndicators
    /*    */                              // These bits control the white LEDs under the touch pad.
    /*    */                              // Note the reduction in functionality for later revisions.
    /*    */                              // Generation 0x03 - Full Functionality
    /*    */                              // Generation 0x04 - Mirrored Only
    /*    */                              // Suggested detection: (HardwareInfo & 0x00FFFF00) == 0X00000400
    /*    */                              //
    /*    */                              // Layout used by PS5:
    /*    */                              // 0x04 - -x- -  Player 1
    /*    */                              // 0x06 - x-x -  Player 2
    /*    */                              // 0x15 x -x- x  Player 3
    /*    */                              // 0x1B x x-x x  Player 4
    /*    */                              // 0x1F x xxx x  Player 5* (Unconfirmed)
    /*    */                              //
    /*    */                              //                        // HW 0x03 // HW 0x04
    /*43.0*/ uint8_t PlayerLight1 : 1;    // x --- - // x --- x
    /*43.1*/ uint8_t PlayerLight2 : 1;    // - x-- - // - x-x -
    /*43.2*/ uint8_t PlayerLight3 : 1;    // - -x- - // - -x- -
    /*43.3*/ uint8_t PlayerLight4 : 1;    // - --x - // - x-x -
    /*43.4*/ uint8_t PlayerLight5 : 1;    // - --- x // x --- x
    /*43.5*/ uint8_t PlayerLightFade : 1; // if low player lights fade in, if high player lights instantly change
    /*43.6*/ uint8_t PlayerLightUNK : 2;
    /*    */
    /*    */ // RGB LED
    /*44  */ uint8_t LedRed;
    /*45  */ uint8_t LedGreen;
    /*46  */ uint8_t LedBlue;
    // Structure ends here though on BT there is padding and a CRC, see ReportOut31
};
static_assert(sizeof(SetStateData) == 47, "struct wrong size");

struct ReportIn01
{
    BTSimpleGetStateData State;
};
static_assert(sizeof(ReportIn01) == 9, "struct wrong size");

struct ReportIn31
{
    union
    {
        BTCRC<77> CRC;
        struct
        {
            // uint8_t ReportID;   // 0x31
            uint8_t HasHID : 1; // Present for packets with state data
            uint8_t HasMic : 1; // Looks mutually exclusive, possible mic data
            uint8_t Unk1 : 2;
            uint8_t SeqNo : 4; // unclear progression
            BTGetStateData State;
        } Data;
    };
};
static_assert(sizeof(ReportIn31) == 77, "struct wrong size");

struct ReportOut31
{
    union
    {
        BTCRC<77> CRC;
        struct
        {
            // uint8_t ReportID;      // 0x31
            uint8_t UNK1 : 1;      // -+
            uint8_t EnableHID : 1; //  | - these 3 bits seem to act as an enum
            uint8_t UNK2 : 1;      // -+
            uint8_t UNK3 : 1;
            uint8_t SeqNo : 4; // increment for every write // we have no proof of this, need to see some PS5 captures
                // uint8_t tag;
            SetStateData State;
        } Data;
    };
};
static_assert(sizeof(ReportOut31) == 77, "struct wrong size");

struct ReportFeature05
{
    union
    {
        BTCRC<40> CRC;
        struct
        {
            // uint8_t ReportID; // 0x05 // does this exist on USB? confirm
            int16_t GyroPitchBias;
            int16_t GyroYawBias;
            int16_t GyroRollBias;
            int16_t GyroPitchPlus;
            int16_t GyroPitchMinus;
            int16_t GyroYawPlus;
            int16_t GyroYawMinus;
            int16_t GyroRollPlus;
            int16_t GyroRollMinus;
            int16_t GyroSpeedPlus;
            int16_t GyroSpeedMinus;
            int16_t AccelXPlus;
            int16_t AccelXMinus;
            int16_t AccelYPlus;
            int16_t AccelYMinus;
            int16_t AccelZPlus;
            int16_t AccelZMinus;
            int16_t Unknown;
        } Data;
    };
};
static_assert(sizeof(ReportFeature05) == 40, "struct wrong size");

struct ReportFeature09
{
    // uint8_t ReportID;     // 0x09
    uint8_t ClientMac[6]; // Right to Left
    uint8_t Hard08;
    uint8_t Hard25;
    uint8_t Hard00;
    uint8_t HostMac[6]; // Right to Left
    uint8_t Pad[4];     // Size according to Linux driver
};
static_assert(sizeof(ReportFeature09) == 19, "struct wrong size");

struct ReportFeature20
{
    union
    {
        BTCRC<63> CRC;
        struct
        {
            // uint8_t ReportID;   // 0x20
            char BuildDate[11]; // string
            char BuildTime[8];  // string
            uint16_t FwType;
            uint16_t SwSeries;
            uint32_t HardwareInfo;    // 0x00FF0000 - Variation
                                      // 0x0000FF00 - Generation
                                      // 0x0000003F - Trial?
                                      // ^ Values tied to enumerations
            uint32_t FirmwareVersion; // 0xAABBCCCC AA.BB.CCCC
            char DeviceInfo[12];
            ////
            uint16_t UpdateVersion;
            char UpdateImageInfo;
            char UpdateUnk;
            ////
            uint32_t FwVersion1; // AKA SblFwVersion
                                 // 0xAABBCCCC AA.BB.CCCC
                                 // Ignored for FwType 0
                                 // HardwareVersion used for FwType 1
                                 // Unknown behavior if HardwareVersion < 0.1.38 for FwType 2 & 3
                                 // If HardwareVersion >= 0.1.38 for FwType 2 & 3
            uint32_t FwVersion2; // AKA VenomFwVersion
            uint32_t FwVersion3; // AKA SpiderDspFwVersion AKA BettyFwVer
                                 // May be Memory Control Unit for Non Volatile Storage
        };
    };
};
static_assert(sizeof(ReportFeature20) == 63, "struct wrong size");

#pragma pack(pop)

class DualSenseController : public IMyGamepad, public NimBLECharacteristicCallbacks
{
private:
    NimBLECharacteristic *_iRptChara01;
    NimBLECharacteristic *_iRptChara31;
    NimBLECharacteristic *_oRptChara31;
    NimBLECharacteristic *_oRptChara32;
    NimBLECharacteristic *_oRptChara33;
    NimBLECharacteristic *_oRptChara34;
    NimBLECharacteristic *_oRptChara35;
    NimBLECharacteristic *_oRptChara36;
    NimBLECharacteristic *_oRptChara37;
    NimBLECharacteristic *_oRptChara38;
    NimBLECharacteristic *_oRptChara39;
    NimBLECharacteristic *_fRptChara05;
    NimBLECharacteristic *_fRptChara08;
    NimBLECharacteristic *_fRptChara09;
    NimBLECharacteristic *_fRptChara20;
    NimBLECharacteristic *_fRptChara22;
    NimBLECharacteristic *_fRptChara80;
    NimBLECharacteristic *_fRptChara81;
    NimBLECharacteristic *_fRptChara82;
    NimBLECharacteristic *_fRptChara83;
    NimBLECharacteristic *_fRptCharaF0;
    NimBLECharacteristic *_fRptCharaF1;
    NimBLECharacteristic *_fRptCharaF2;
    uint8_t mac[6] = {};
    uint32_t hostTimeStamp = 0;
    uint8_t ipRptMode;
    uint8_t isIMUEnable = 0;

    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onNotify(NimBLECharacteristic *pCharacteristic) override;
    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override;

    void _initStaticCharaValues();
    void _enableIMU(bool enable);
    void _handleReportOut31(struct ReportOut31 *rpt);
    void _handleReportIn01(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt);
    void _handleReportIn31(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt);

public:
    DualSenseController(const std::string &deviceName);
    ~DualSenseController();
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