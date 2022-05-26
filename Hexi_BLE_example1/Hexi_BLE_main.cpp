/**
 * @file Hexi_BLE_main.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @brief Example program for the KW40Z processor
 *      using BLE and communicating with a smartphone
 * @version 0.2
 * @date 2022-03-04
 * 
 * @note based on Mbed program
 * 
 * last updated 2022-05-25
 * 
 */

/* get libraries */
#include "Hexi_BLE_main.h"
#include "mbed.h"
#include "Hexi_KW40Z.h"
#include "HTU21D.h"
#include "FXOS8700.h"
#include "FXAS21002.h"
#include "MAX30101.h"
#include "TSL2561.h"
#include "hexi_battery.h"
#include "Hexi_OLED_SSD1351.h"
#include "string.h"

/* define objects */

RtosTimer hapticTimer(stopHaptics, osTimerOnce);
// Ticker hapticTimer;
Thread txThread();

DigitalOut haptics(PTB9);
DigitalOut redLed(LED1);
DigitalOut blueLed(LED3);
DigitalOut pwr1v8(PTA29);
DigitalOut pwr3v3(PTC13);
DigitalOut pwr15v(PTB12);

KW40Z kw40z(PTE24, PTE25);

HTU21D htu(PTB1,PTB0);
FXOS8700 accel(PTC11, PTC10);
FXOS8700 mag(PTC11, PTC10);
FXAS21002 gyro(PTC11,PTC10);
I2C i2cBus(PTB1, PTB0); // helps control hr
MAX30101 hr(i2cBus);

HexiwearBattery bat();

SSD1351 oled(PTB22, PTB21, PTC13, PTB20, PTE6, PTD15);

float accel_data[3];
float mag_data[3];
float gyro_data[3];
int cTemp, humd;
int hr, light = 0;
uint8_t battery;

/** implementation **/

int main() {
    setup()
    txThread.start(txTask);
    while (true) {
        ThisThread::wait(50);
    }
}


void setup() {
    pwr1v8 = 1;
    pwr3v3 = 1;
    pwr15v = 0;
    kw40z.attach_buttonLeft(&btnLeft);
    kw40z.attach_buttonRight(&btnRight);
    kw40z.attach_passkey(&passKey);
    // setup all sensors
    accel.accel_config();
    mag.mag_config();
    gyro.gyro_config();
    bat.sensorOn();
    hrInter.fall(interHdlr);
    MAX30101::InterruptBitField_u interruptStatus;
    interruptStatus.all = 0xFF;
    hr.enableInterrupts(interruptStatus);
    // OLED setup
    oled.DimScreenON();
    oled.FillScreen(COLOR_BLACK);
    oled_text_properties_t txtProps = { 0 };
    oled.GetTextProperties(&txtProps);

}


void txTask() {
    while (1) {
        updateSensors();
        kw40z.SendSetApplicationMode(GUI_CURRENT_APP_SENSOR_TAG);
        // send sensor data over BLE
        kw40z.SendBatteryLevel( battery );
        kw40z.SendAmbientLight( 0 );
        kw40z.SendTemperature( cTemp );
        kw40z.SendHumidity(humd);
        kw40z.SendAccel(accel_data[0], accel_data[1], accel_data[2]);
        kw40z.SendMag(mag_data[0], mag_data[1], mag_data[2]);
        kw40z.SendGyro(gyro_data[0], gyro_data[1], gyro_data[2]);
        // kw40z.SendHeartRate();

        ThisThread::wait(1000);
    }
}


void updateSensors() {
    if ( bat.isBatteryCharging() ) {
        battery = 100;
    }
    else {
        battery = bat.readLevelPercent()
    }
    cTemp = htu.getTemp();
    humd  = htu.getHumidity();
    accel.acquire_accel_data_g(&accel_data);
    mag.acquire_mag_data_uT(&mag_data);
    gyro.acquire_gyro_data_dps(&gyro_data);

}


void startHaptics() {
    hapticTimer.start(50);
    haptics = 1;
}

void stopHaptics() {
    haptics = 0;
    hapticTimer.stop();
}



/**
 * gathers data from heart rate monitor
 */
void readHR() {
    MAX30101::InterruptBitField_u interruptStatus;
    hr.getInterruptStatus(interruptStatus);
    
    if (interruptStatus.bits.pwr_rdy == 0x1) {
        // Soft reset
        MAX30101::ModeConfiguration_u modeConf;
        modeConf.all = 0;
        modeConf.bits.reset = 1;
        hr.setModeConfiguration(modeConf);
        wait_us(10);

        // Configure FIFO
        MAX30101::FIFO_Configuration_u fifoConf;
        hr.getFIFOConfiguration(fifoConf);
        // Set LED power
        hr.setLEDPulseAmplitude(MAX30101::LED1_PA, 0x0C);
        hr.setLEDPulseAmplitude(MAX30101::ProxModeLED_PA, 0x19);
        // led set
        
        MAX30101::SpO2Configuration_u spo2Conf;
        hr.getSpO2Configuration(spo2Conf);
        spo2Conf.bits.led_pw = MAX30101::PW_1;
        spo2Conf.bits.spo2_sr = MAX30101::SR_100_Hz;
        hr.setSpO2Configuration(spo2Conf);
        hr.getSpO2Configuration(spo2Conf);
        
        // Proximity settings
        hr.setProxIntThreshold(0x14);
        
        // Enable HR mode
        modeConf.all = 0;
        modeConf.bits.mode = MAX30101::HeartRateMode;
        hr.setModeConfiguration(modeConf);
        // mode set
    }
    // Proximity Triggered
    if (interruptStatus.bits.prox_int == 0x1) {
    }
    // PPG Ready
    if (interruptStatus.bits.ppg_rdy == 0x1) {
        mask_ppg = 1;
    }
    
    if (interruptStatus.bits.a_full == 0x1) {
        uint8_t data[FIFO_DATA_MAX];
        uint16_t readBytes = 0;
        hr.readFIFO(MAX30101::OneLedChannel, data, readBytes);
        
        for (uint16_t i = 0; i < readBytes; i += 3) {
            uint8_t sample[4] = {0};
            sample[0] = data[i + 2];
            sample[1] = data[i + 1];
            sample[2] = data[i];
            
            num = *(uint32_t *) sample;
            if (num < 310000) {
                realHeartRate = 0;
            }
            else {
                realHeartRate = (num - 310000)/100;
            }
        }
    }
    interruptStatus.all = 0xFF;
    if (mask_ppg == 1) {
        interruptStatus.bits.ppg_rdy = 0;
    }
    hr.enableInterrupts(interruptStatus);
    #ifdef DEBUG
    printf("HR: %d\r\n", realHeartRate);
    #endif // DEBUG
}
