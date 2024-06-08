#include "IMyGamepad.h"
#include "NimBLEDevice.h"
#include "ble_uuid.h"
#include "esp_log.h"
#include "helper.h"
static const char *TAG = "mygamepad";

class MyBattCharaCallback : public NimBLECharacteristicCallbacks
{
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
    {
        //订阅了，发一次电量
        if (subValue & 0x01)
        {
            uint8_t level = getBatteryLevel();
            pCharacteristic->notify(&level, 1, true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
        }
    }
};

IMyGamepad::IMyGamepad(const std::string &deviceName)
{
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT); //太低传输延迟会变大

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(this);
    pHidDev = new NimBLEHIDDevice(pServer);
    NimBLEDevice::setDeviceName(deviceName);

    int n = NimBLEDevice::getNumBonds();
    ESP_LOGI(TAG, "Bond count: %d", n);
    for (int i = 0; i < n; i++)
    {
        ESP_LOGI(TAG, "Bond addr: %s", NimBLEDevice::getBondedAddress(i).toString().c_str());
    }

    _battChara = pHidDev->batteryService()->getCharacteristic((uint16_t)UUID_BATT_LEVEL);
    _battChara->setCallbacks(new MyBattCharaCallback());

    // 想设置下hid product name 但是下面那些都没用。
    // static const char *name1 = "qingwa2009";
    // static const char *name2 = "Software Revision";
    // static const char *name3 = "Hardware Revision";
    // static const char *name4 = "Firmware Revision";
    // static const char *name5 = "Info Model Number";
    // NimBLECharacteristic *_chara;
    // _chara = pHidDev->deviceInfo()->createCharacteristic((uint16_t)UUID_MANUFACTURER_NAME, NIMBLE_PROPERTY::READ);
    // _chara->setValue(std::string(name1));
    // _chara = pHidDev->deviceInfo()->createCharacteristic((uint16_t)UUID_SOFTWARE_REV, NIMBLE_PROPERTY::READ);
    // _chara->setValue(std::string(name2));
    // _chara = pHidDev->deviceInfo()->createCharacteristic((uint16_t)UUID_HARDWARE_REV, NIMBLE_PROPERTY::READ);
    // _chara->setValue(std::string(name3));
    // _chara = pHidDev->deviceInfo()->createCharacteristic((uint16_t)UUID_FIRMWARE_REV, NIMBLE_PROPERTY::READ);
    // _chara->setValue(std::string(name4));
    // _chara = pHidDev->deviceInfo()->createCharacteristic((uint16_t)UUID_MODEL_NUMBER, NIMBLE_PROPERTY::READ);
    // _chara->setValue(std::string(name5));
}

IMyGamepad::~IMyGamepad()
{
}
void IMyGamepad::start(bool powBoostMode)
{
    if (powBoostMode)
    {
        SEND_PERIOD_MS = BOOST_SEND_PERIOD_MS;
        DESIRED_MIN_CONN_INTERVAL = BOOST_DESIRED_MIN_CONN_INTERVAL;
        DESIRED_MAX_CONN_INTERVAL = BOOST_DESIRED_MAX_CONN_INTERVAL;
        DESIRED_SLAVE_LATENCY = BOOST_DESIRED_SLAVE_LATENCY / 2;

        NimBLEDevice::setPower(ESP_PWR_LVL_P21, ESP_BLE_PWR_TYPE_DEFAULT);
    }

    pHidDev->startServices();
    pServer->advertiseOnDisconnect(true);
    pServer->start();

    setLEDScanning(1);

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NimBLEUUID((uint16_t)UUID_HID_SERV));
    pAdvertising->setAppearance(HID_GAMEPAD);
    pAdvertising->start(MAX_ADVERTISING_TIME_MS);
}
void IMyGamepad::stop()
{
    pServer->advertiseOnDisconnect(false);
    pServer->stopAdvertising();
    for (auto cnnid : pServer->getPeerDevices())
    {
        pServer->disconnect(cnnid, BLE_ERR_RD_CONN_TERM_PWROFF);
    }
}
size_t IMyGamepad::getConnectedCount()
{
    return pServer->getConnectedCount();
}
void IMyGamepad::notifyBattLevel(uint8_t level)
{
    if (_battChara->getSubscribedCount())
    {
        pHidDev->setBatteryLevel(level);
        _battChara->notify(true, BLE_HCI_LE_CONN_HANDLE_MAX + 1);
    }
}
void IMyGamepad::setDeadZone(int deadx, int deady, int deadz, int deadrz)
{
    dead_x = deadx;
    dead_y = deady;
    dead_z = deadz;
    dead_rz = deadrz;
}
bool IMyGamepad::isKeyStateChange(KeyMatrix_t *keys, uint8_t btnXbox, uint8_t btnSelect, int x, int y, int z, int rz, int lt, int rt)
{
    static uint16_t keys0 = 0;
    static uint8_t btnXbox0 = 0, btnSelect0 = 0;
    static uint8_t repeatSendCount = 0;

    if (keys0 == *(uint16_t *)keys &&
        (x > (50 - dead_x) && x < (50 + dead_x)) &&
        (y > (50 - dead_y) && y < (50 + dead_y)) &&
        (z > (50 - dead_z) && z < (50 + dead_z)) &&
        (rz > (50 - dead_rz) && rz < (50 + dead_rz)) &&
        lt == 0 && rt == 0 &&
        btnXbox0 == btnXbox && btnSelect0 == btnSelect)
    {
        if (repeatSendCount > 10) //无按键时，多重发几次，避免主机漏了
        {
            return false;
        }
        repeatSendCount++;
    }
    else
    {
        repeatSendCount = 0;
    }

    keys0 = *(uint16_t *)keys;
    btnXbox0 = btnXbox;
    btnSelect0 = btnSelect;

    return true;
}

int IMyGamepad::dirKey2DPadValue(int up, int right, int down, int left)
{
    uint8_t dir = up | (right << 1) | (down << 2) | (left << 3);
    switch (dir)
    {
    case 1:
    case 11:
        return 1;
    case 2:
    case 7:
        return 3;
    case 3:
        return 2;
    case 4:
    case 14:
        return 5;
    case 6:
        return 4;
    case 8:
    case 13:
        return 7;
    case 9:
        return 8;
    case 12:
        return 6;
    default:
        return 0;
    }
}

void IMyGamepad::clearBond()
{
    NimBLEDevice::deleteAllBonds();
    ESP_LOGI(TAG, "clear all bond!!!");
}

void IMyGamepad::onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
{
    ESP_LOGI(TAG, "Client connect: %s\n", connInfo.getAddress().toString().c_str());

    /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 3x interval time for best results.
         */
    pServer->updateConnParams(connInfo.getConnHandle(),
                              DESIRED_MIN_CONN_INTERVAL, DESIRED_MAX_CONN_INTERVAL,
                              DESIRED_SLAVE_LATENCY, DEFAULT_SUPERVISION_TIMEOUT);

    setLEDScanning(0);
};

void IMyGamepad::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
{
    ESP_LOGI(TAG, "Client disconnected reason: %d\n", reason);
    // NimBLEDevice::startAdvertising();
    setLEDScanning(1);
};

void IMyGamepad::onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo)
{
    ESP_LOGI(TAG, "MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
};

/********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
uint32_t IMyGamepad::onPassKeyRequest()
{
    ESP_LOGI(TAG, "Server Passkey Request\n");
    /** This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
    return 123456;
};

bool IMyGamepad::onConfirmPIN(uint32_t pass_key)
{
    ESP_LOGI(TAG, "The passkey YES/NO number: %" PRIu32 "\n", pass_key);
    /** Return false if passkeys don't match. */
    return true;
};

void IMyGamepad::onAuthenticationComplete(NimBLEConnInfo &connInfo)
{
    /** Check that encryption was successful, if not we disconnect the client */
    if (!connInfo.isEncrypted())
    {
        NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
        ESP_LOGI(TAG, "Encrypt connection failed - disconnecting client\n");
        return;
    }
    ESP_LOGI(TAG, "Authentication Complete!");
};

uint8_t getRptID(NimBLECharacteristic *pCharacteristic)
{
    NimBLEDescriptor *desc = pCharacteristic->getDescriptorByUUID((uint16_t)UUID_GATT_REPORT_REF);
    uint8_t id = ((const NimBLEAttValue &)(desc->getValue(nullptr)))[0];
    return id;
}
