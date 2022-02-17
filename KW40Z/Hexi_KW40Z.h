/** 
 * @file Hexi_KW40Z.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Feb 17, 2022 
 * @brief headers of the KW40Z for
 *      Hexiwear development in Mbed OS 5.12
 *      with customized outputs
 * 
 * NXP License below
 * 
 * BLE KW40Z Driver for Hexiwear
 * This file contains BLE and Touch Buttons driver functionality for Hexiwear
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NXP, nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * visit: http://www.mikroe.com and http://www.nxp.com
 *
 * get support at: http://www.mikroe.com/forum and https://community.nxp.com
 *
 * Project HEXIWEAR, 2015
 * 
 * last updated Feb 17, 2022
 */

#ifndef HG_HEXI_KW40Z
#define HG_HEXI_KW40Z

#include "mbed.h"
#include "rtos.h"

#define LIB_DEBUG
//#define RAW_DEBUG

#define gHostInterface_startByte1               0x55
#define gHostInterface_startByte2               0xAA
#define gHostInterface_trailerByte              0x45
#define gHostInterface_rxConfirmMask            0x01
#define gHostInterface_txPacketMask             0x10

#define gHostInterface_dataSize                 23
#define gHostInterface_headerSize               4

#define gHostInterface_retransmitCount          3
#define gHostInterface_retransmitTimeout        100

#define gHostInterface_TxConfirmationEnable     1 // send confirmation when receive packet
#define gHostInterface_RxConfirmationEnable     1 // wait on confirmation from remote side (do retransmit)

/** HEXIWEAR firmware version */
#define HEXIWEAR_VERSION_PATCH ( 0 )
#define HEXIWEAR_VERSION_MINOR ( 0 )
#define HEXIWEAR_VERSION_MAJOR ( 2 )

/** packet types */
typedef enum
{
    packetType_pressUp          = 0, /**< touch press up */
    packetType_pressDown        = 1, /**< touch press down */
    packetType_pressLeft        = 2, /**< touch press left */
    packetType_pressRight       = 3, /**< touch press right */
    packetType_slide            = 4, /**< touch slide */
    
    packetType_batteryLevel     = 5, /**< battery Service */
    
    packetType_accel            = 6, /**< motion service */
    packetType_ambiLight        = 7, /**< weather service */
    packetType_pressure         = 8, /**< weather service */
    
    
    packetType_gyro             = 9,  /**< motion service */
    packetType_temperature      = 10, /**< weather service */
    packetType_humidity         = 11, /**< weather service */
    packetType_magnet           = 12, /**< motion service */
    
    packetType_heartRate        = 13, /**< health service */
    packetType_steps            = 14, /**< health service */
    packetType_calories         = 15, /**< health service */
    
    /* Alert Service */
    packetType_alertIn          = 16, /**<  incoming alerts */
    packetType_alertOut         = 17, /**<  outcoming alerts */
    
    packetType_passDisplay      = 18, /**< key display type */
    
    /* OTAP procedure types */
    packetType_otapKW40Started  = 19,
    packetType_otapMK64Started  = 20,
    packetType_otapCompleted    = 21,
    packetType_otapFailed       = 22,
    
    /* active buttons types */
    packetType_buttonsGroupToggleActive = 23,
    packetType_buttonsGroupGetActive    = 24,
    packetType_buttonsGroupSendActive   = 25,
    
    /* Turn off/on bluetooth advertising */
    packetType_advModeGet    = 26,
    packetType_advModeSend   = 27,
    packetType_advModeToggle = 28,
    
    packetType_appMode       = 29, /**< app mode service */
    
    /* Link State */
    packetType_linkStateGet  = 30, /**< connected */
    packetType_linkStateSend = 31, /**< disconnected */
    
    packetType_notification  = 32, /**< notifications */
    
    packetType_buildVersion  = 33, /**< build version */
    
    packetType_sleepON       = 34, /**< sleep ON */
    packetType_sleepOFF      = 35, /**< sleep OFF */

    // TODO add on here
    packetType_kalman        = 100,/**< Kalman Filter service */
    
    packetType_OK            = 255 /**< OK packet */
} hostInterface_packetType_t;

/** data-packet structure */
typedef struct
{
    /* NOTE: Size of struct must be multiplier of 4! */
    uint8_t start1;
    uint8_t start2;
    hostInterface_packetType_t type;
    uint8_t length;
    uint8_t data[gHostInterface_dataSize + 1];
} hostInterface_packet_t;

/** incoming alert types */
typedef enum
{
    alertIn_type_notification        = 1,
    alertIn_type_settings            = 2,
    alertIn_type_timeUpdate          = 3,
} hostInterface_alertIn_type_t;

/** current app enum */
typedef enum
{
  GUI_CURRENT_APP_IDLE                = 0, /**< no app active */
  GUI_CURRENT_APP_SENSOR_TAG          = 2, /**< sensor tag */
  GUI_CURRENT_APP_HEART_RATE          = 5, /**< heart rate */
  GUI_CURRENT_APP_PEDOMETER           = 6  /**< Pedometer */
} gui_current_app_t;

typedef void (*button_t)(void);
typedef void (*alert_t)(uint8_t *data, uint8_t length);
//typedef void (*passkey_t)(uint8_t *data);
typedef void (*passkey_t)(void);
typedef void (*notifications_t)(uint8_t eventId, uint8_t categoryId);

typedef struct name
{
    uint8_t ver_patchNumber;
    uint8_t ver_minorNumber;
    uint8_t ver_majorNumber;

} hexiwear_version_t;

class KW40Z{

public:

    /**
    * Create a Hexiwear BLE KW40Z Driver connected to the UART pins
    *    
    * @param txPin UART TX pin
    * @param rxPin UART RX pin
    */
    KW40Z(PinName txPin,PinName rxPin);
    
    /**
    * Destroy the Hexiwear instance
    */   
    ~KW40Z();
    
    void attach_buttonUp(button_t btnFct);
    void attach_buttonDown(button_t btnFct);
    void attach_buttonLeft(button_t btnFct);
    void attach_buttonRight(button_t btnFct);
    void attach_buttonSlide(button_t btnFct);
    
    void attach_alert(alert_t alertFct);
    void attach_passkey(passkey_t passkeyFct);
    void attach_notifications(notifications_t notFct);

    void SendBatteryLevel(uint8_t percentage);
    void SendAccel(int16_t x, int16_t y, int16_t z);
    void SendGyro(int16_t x, int16_t y, int16_t z);
    void SendMag(int16_t x, int16_t y, int16_t z);
    void SendAmbientLight(uint8_t percentage);
    void SendTemperature(uint16_t celsius);
    void SendHumidity(uint16_t percentage);
    void SendPressure(uint16_t pascal);
    void SendHeartRate(uint8_t rate);
    void SendSteps(uint16_t steps);
    void SendCalories(uint16_t calories);
    void SendAlert(uint8_t *pData, uint8_t length);
    void SendSetApplicationMode(gui_current_app_t mode);
    void SendGetVersion(void);

    void SendKalman(int16_t kalman[3]);
    
    void ToggleTsiGroup(void);
    void ToggleAdvertisementMode(void);
    
    uint8_t GetTsiGroup(void);
    uint8_t GetAdvertisementMode(void);
    uint8_t GetLinkState(void);
    hexiwear_version_t GetVersion(void);
    
    uint32_t GetPassKey(void); 

private:
#ifdef LIB_DEBUG
    RawSerial pc;
#endif

    RawSerial device;
    Thread rxThread;  
    Thread mainThread;
    
    hostInterface_packet_t hostInterface_rxPacket;
    hostInterface_packet_t hostInterface_txPacket;
    
    button_t buttonUpCb;
    button_t buttonDownCb;
    button_t buttonLeftCb;
    button_t buttonRightCb;
    button_t buttonSlideCb;
    
    alert_t alertCb;
    passkey_t passkeyCb;
    notifications_t notificationsCb;
    
    uint8_t * rxBuff;
    bool confirmReceived;
    
    hexiwear_version_t kw40_version;
    uint8_t activeTsiGroup;
    uint8_t advertisementMode;
    uint8_t linkState;
    uint32_t bondPassKey; 
    
    MemoryPool<hostInterface_packet_t, 50> mpool;
    Queue<hostInterface_packet_t, 50> queue;

    void mainTask(void);
    void rxTask(void);
    
    void ProcessBuffer();
    void ProcessPacket(hostInterface_packet_t * packet);
    void SendPacket(hostInterface_packet_t * txPacket, bool confirmRequested);
    void SendInternal(hostInterface_packet_t * txPacket);
    void SearchStartByte();
    
    void SendPacketOK(void);
    void SendGetActiveTsiGroup(void);
    void SendGetAdvertisementMode(void);
    void SendGetLinkState(void);
    
#ifdef LIB_DEBUG
    void DebugPrintPacket(hostInterface_packet_t * packet);
#endif
};

#endif
