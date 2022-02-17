/**
 * @file KW40Z.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Feb 10, 2022
 * @brief implementation of the KW40Z for
 *      Hexiwear development in Mbed OS 5.12
 *      with customized outputs
 * 
 * @note to be used with HexiIMU
 * @note consider pull from original Mbed code
 * @note add comments to know what to do
 * 
 * last updated: Feb 17, 2022
 */

#pragma once

#ifndef KW40Z_H
#define KW40Z_H

#include "mbed.h"
#include "rtos.h"
// #include "RawSerial.h"

// Comment in or out based on if you want to see
// the package in the serial monitor
#define DEBUG
#define LIB_DEBUG

/** Constants and Structures **/

#define START_THREAD                    1

/* communication */

#define kwHostInterface_startByte1      0x55
#define kwHostInterface_startByte2      0xAA
#define kwHostInterface_endByte         0x45

#define kwHostInterface_rxMask          0x01
#define kwHostInterface_txMask          0x10
#define kwHostInterface_txConfirm       0x01
#define kwHostInterface_rxConfirm       0x01

#define kwHostInterface_dataSize        0x17
#define kwHostInterface_headerSize      0x04

#define kwHostInterface_retransmitMax   3
#define kwHostInterface_retransmitTO    100     // time-out

/* version */

#define hexi_version_major  2
#define hexi_version_minor  0
#define hexi_version_patch  0

/* packets */

typedef enum {
    packet_pressUp      = 0,
    packet_pressDown    = 1,
    packet_pressLeft    = 2,
    packet_pressRight   = 3,
    packet_pressSlide   = 4,

    packet_temp         = 5,
    packet_humd         = 6,
    packet_pressure     = 7,

    packet_accel        = 8,
    packet_mag          = 9,
    packet_gyro         = 10,

    packet_heartRate    = 11,
    packet_steps        = 12,
    packet_cal          = 13,
    
    packet_battery      = 14,

    packet_alertIn      = 15,
    packet_alertOut     = 16,
    packet_pass,

    packet_otapkw40     = 18,
    packet_otapmk64     = 19,
    packet_otapComplete = 20,
    packet_otapFail     = 21,

    packet_btnGroupToggleActive = 22,
    packet_btnGroupGetActive    = 23,
    packet_btnGroupSendActive   = 24,

    packet_advToggle    = 25,
    packet_advGet       = 26,
    packet_advSend      = 27,

    packet_app          = 28,

    packet_linkGet      = 29,
    packet_linkSend     = 30,

    packet_notification = 31,

    packet_buildVer     = 32,

    packet_sleepOn      = 33,
    packet_sleepOff     = 34,
    // TODO

    packet_kalman       = 100,

    packet_OK           = 255
} kwHostInterface_packetType_t;

/* structure that holds BLE packet data */
typedef struct {
    uint8_t start1;
    uint8_t start2;
    kwHostInterface_packetType_t type;
    uint8_t length;
    uint8_t data[kwHostInterface_dataSize+1];
} kwHostInterface_packet_t;

/* alert types */

/* active alert signal */
typedef enum {
    alertIn_idle            = 0,
    alertIn_notification    = 1,
    alertIn_settings        = 2,
    alertIn_timeUpdate      = 3,
} kwHostInterface_alertIn_t;

/* active gui element */
typedef enum {
    kwHostInterface_gui_idle        = 0,    // not active
    kwHostInterface_gui_sensor_tag  = 2,
    kwHostInterface_gui_heart_rate  = 3,
    kwHostInterface_gui_pedometer   = 4,
    kwHostInterface_gui_kalman      = 10,
} kwHostInterface_gui_t;

/* structure that holds Hexiwear Firmware Version */
typedef struct verion {
    uint8_t verMajor;
    uint8_t verMinor;
    uint8_t verPatch;
} hexi_version_t;

/* custom types */

typedef void (*button_t)(void);
typedef void (*alert_t)(uint8_t *data, uint8_t size);
typedef void (*passkey_t)(void);
typedef void (*notification_t)(uint8_t eventId, uint8_t categoryId);


/** Class Prototype **/

class KW40Z {

    /**
     * Creates the KW40Z Driver instance
     * connecting to the UART pins
     * @param txPin TX pin on UART
     * @param rxPin RX pin on UART
     */
    KW40Z(PinName txPin, PinName rxPin);

    /**
     * Creates the KW40Z Driver instance
     * connecting to the default UART pins
     */
    KW40Z();

    /**
     * Destroys KW40Z instance
     */
    ~KW40Z();

    void attach_btnUp(button_t btnFunct);
    void attach_btnDown(button_t btnFunct);
    void attach_btnLeft(button_t btnFunct);
    void attach_btnRight(button_t btnFunct);
    void attach_btnSlide(button_t btnFunct);

    void SendBattery(uint8_t percent);
    void SendAccel(uint8_t accel_data[3]);
    void SendMag(uint8_t mag_data[3]);
    void SendGyro(uint8_t gyro_data[3]);
    void SendTemp(uint16_t celsius);
    void SendHumd(uint16_t percent);
    void SendPressure(uint16_t pascal);
    void SendHeartRate(uint16_t hr);

    void SendSteps(uint16_t steps);
    void SendCalories(uint16_t cal);
    
    void SendAlert(uint8_t *pData, uint8_t length);
    void SendSetApplicationMode(kwHostInterface_gui_t mode);

    // TODO additional BLE functions
    void SendKalman(uint8_t kal_data[3]);

    hexi_version_t GetVersion();

    void ToggleTsiGroup();
    void ToggleAdvertisementMode();

    void GetTsiGroup();
    void GetAdvertisementMode();
    void GetLinkState();


private:

    RawSerial serial;
    Thread mainThread;
    Thread rxThread;

    button_t btnUp;
    button_t btnDown;
    button_t btnLeft;
    button_t btnRight;
    button_t btnSlide;

    alert_t alert;
    passkey_t passkey;
    notification_t notif;

    kwHostInterface_packet_t hostInterface_rx;
    kwHostInterface_packet_t hostInterface_tx;

    uint8_t *rxBuffer;
    bool confirmReceived;

    MemoryPool<kwHostInterface_packet_t, 50> mpool;
    Queue<kwHostInterface_packet_t, 50> queue;

    hexi_version_t kwVersion;
    uint8_t activeTsiGroup;
    uint8_t advertisementMode;
    uint8_t linkState;
    uint32_t bondPassKey;

    void rxTask();
    void mainTask();

    void ProcessBuffer();
    void ProcessReceivedPacket(kwHostInterface_packet_t *rxPacket);
    void SendPacket(kwHostInterface_packet_t *txPacket, bool confirmReq);
    void SearchStartByte();

    void SendPacketOK();
    void SendGetActiveTsiGroup();
    void SendGetLinkState();
    void SendGetVersion();

    // TODO add additional useful variables, 
    // memory elements, and functions

    static void rxStarter(void *p);
    static void mainStarter(void *p);

#ifdef LIB_DEBUG

    void DebugPrintRxPacket(void *p);
    void DebugPrintRxPacket(void *p);

#endif // LIB_DEBUG
};


#endif // KW40Z