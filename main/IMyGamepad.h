#ifndef _I_MY_GAMEPAD_H_
#define _I_MY_GAMEPAD_H_
#include "pcbConfig.h"
#include "NimBLEHIDDevice.h"

#define MAX_ADVERTISING_TIME_MS 0 //0 for infinite
// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 12
// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 16
// Slave latency to use if automatic parameter update request is enabled
// 0 ~ ((connSupervisionTimeout / (connIntervalMax*2)) -1).
#define DEFAULT_DESIRED_SLAVE_LATENCY 0
// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_SUPERVISION_TIMEOUT 300

#define DEFAULT_SEND_PERIOD_MS 14 //发送间隔ms

#define BOOST_DESIRED_MIN_CONN_INTERVAL 6
#define BOOST_DESIRED_MAX_CONN_INTERVAL 6
#define BOOST_DESIRED_SLAVE_LATENCY 0
#define BOOST_DESIRED_SLAVE_LATENCY 0
#define BOOST_SEND_PERIOD_MS 6 //发送间隔ms

#define BUILD_UINT16(loByte, hiByte) \
    ((uint16_t)(((loByte)&0x00FF) + (((hiByte)&0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a)&0xFF)

class IMyGamepad : public NimBLEServerCallbacks
{
private:
    NimBLEServer *pServer;
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override;
    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override;
    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override;
    uint32_t onPassKeyRequest() override;
    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override;
    bool onConfirmPIN(uint32_t pin) override;

protected:
    NimBLEHIDDevice *pHidDev;
    NimBLECharacteristic *_battChara;
    bool isKeyStateChange(KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt);
    //0~8 0没输入
    int dirKey2DPadValue(int up, int right, int down, int left);
    int dead_x, dead_y, dead_z, dead_rz;
    int SEND_PERIOD_MS = DEFAULT_SEND_PERIOD_MS;
    int DESIRED_MIN_CONN_INTERVAL = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    int DESIRED_MAX_CONN_INTERVAL = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    int DESIRED_SLAVE_LATENCY = DEFAULT_DESIRED_SLAVE_LATENCY;

public:
    IMyGamepad(const std::string &deviceName);
    ~IMyGamepad();
    /*x,y,z,rz,lt,rt: 0~100*/
    virtual void update(uint32_t dt, KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt) = 0;
    void start(bool powBoostMode = false);
    void stop();
    size_t getConnectedCount();
    void notifyBattLevel(uint8_t level);
    void clearBond();
    void setDeadZone(int deadx, int deady, int deadz, int deadrz);
};

uint8_t getRptID(NimBLECharacteristic *pCharacteristic);

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
}
#endif

#endif