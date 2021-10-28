/**
 * @file main.cpp
 * @author Andrew W
 * @date created 10/26/21
 *
 * @brief implements Hexiwear sensors in an event driven manner
 *
 * last updated: 10/27/21
 */

#include "mbed.h"
// import sensors
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_KW40Z.h"
#include "HTU21D.h"
#include "FXOS8700.h"
#include "FXAS21002.h"
#include "MAX30101.h"
// import other header files
#include "images.h"
// import std libraries
#include <string>


#define FIFO_DATA_MAX 288

/* instantiate sensors and enables */
// SSD1351 OLED Driver (MOSI,SCLK,POWER,CS,RST,DC)
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);
KW40Z kw40z(PTE24, PTE25);
HTU21D htu(PTB1,PTB0); // HTU21D Sensor
FXOS8700 accel(PTC11, PTC10);
FXOS8700 mag(PTC11, PTC10);
FXAS21002 gyro(PTC11,PTC10);
I2C i2cBus(PTB1, PTB0);
MAX30101 hr(i2cBus);

Serial pc(USBTX, USBRX);    // Serial interface at 9600 baud; not needede
DigitalOut powerEN (PTB12); // Power Enable HTU21D Sensor
DigitalOut led(LED1);       // red LED
DigitalOut pwr1v8(PTA29);
DigitalOut pwr3v3b(PTC13);
DigitalOut pwr15v(PTB12);
DigitalOut haptic(PTB9);
InterruptIn hrInter(PTB18);
Ticker hapticTimer;


/* create event items */
EventQueue evq(32 * EVENTS_EVENT_SIZE);
// TODO create events to exe later

/* create threads */
Thread the;
// Thread the2;


/** declare structures */

/**
 * used to keep track of which screen the device is on
 */
typedef enum {
    HTU_screen,
    ACCEL_screen,
    GYRO_screen,
    HR_screen,
} screens_t;

/** declare variables */

// used for printing to OLED
char txt[20];
oled_text_properties_t txtProps = { 0 };
const int8_t nScreen = 4; // 4 after HR finished
int8_t cScreen; // current screen
int8_t lScreen; // last screen

// for HTU
int cTemp,
    // fTemp,
    // kTemp,
    humid;
// for FXOS
float accel_data[3];
float accel_rms=0.0;
float mag_data[3];  
float mag_rms=0.0;
float gyro_data[3]; // Storage for the data from the sensor
float gyro_rms=0.0; // RMS value from the sensor
float gx, gy, gz;   // value from the sensor to be displayed

// for HR
int realHeartRate;
// float calorie;
int mask_ppg = 0;
uint32_t count = 0;
uint32_t num;
// uint8_t testsignal = 60;

// for other
// uint8_t battery = 100;
// uint8_t light = 0;


/* function prototypes */
void oled_printLabel(uint8_t *txt, uint8_t xCrd, uint8_t yCrd);
void oled_printTextBox(uint8_t *txt, ...); // TODO: complete from functs

void systemOFF();
void flipLED();

void btnLeft();
void btnRight();
void hapticON();
void hapticOFF();

void showHTU();
void showACCEL();
void showGYRO();

void showHR();
void interHdlr();
void readHR();

void startHaptic();
void stopHaptic();


/* implementation */

int main()
{
    /** setup sensors */
    led = 1;
    powerEN = 0;
    pwr1v8 = 1;
    pwr3v3b = 1;
    pwr15v = 0;
    /* Configure Accelerometer FXOS8700, Magnetometer FXOS8700 */
    accel.accel_config();
    mag.mag_config();
    /* do now to increase speed later */
    accel.acquire_accel_data_g(accel_data);
    mag.acquire_mag_data_uT(mag_data);
    gyro.gyro_config();
    gyro.acquire_gyro_data_dps(gyro_data);
    /* Configure for HR */
    pwr1v8 = 1;
    pwr3v3b = 1;
    pwr15v = 0;
    hrInter.fall(interHdlr);
    // hrInter.enable_irq();
    MAX30101::InterruptBitField_u interruptStatus;
    interruptStatus.all = 0xFF;
    hr.enableInterrupts(interruptStatus);

    /* setup buttons */
    kw40z.attach_buttonLeft(&btnLeft);
    kw40z.attach_buttonRight(&btnRight);
    kw40z.attach_buttonUp(&systemOFF);
    kw40z.attach_buttonDown(&flipLED);

    /* start OLED */
    oled.PowerON();
    oled.DimScreenON();

    /* opening spash screen */
    const uint8_t *logo_img = hexiwear_logo_bmp;
    oled.DrawImage( logo_img, 0, 0 );
    wait(5.0f); // wait 5s

    oled.FillScreen(COLOR_BLACK);
    wait_ms(20);

    /* setup next screen */
    cScreen = HTU_screen;
    lScreen = -1;

    oled.GetTextProperties( &txtProps );

    /* setup events */

    the.start(callback(&evq, &EventQueue::dispatch_forever));
    // the.join();

    while (true) {

        if ( HTU_screen == cScreen ) {
            evq.call(showHTU);
            // showHTU();
        }
        else if ( ACCEL_screen == cScreen ) {
            evq.call(showACCEL);
            // showACCEL();
        }
        else if ( GYRO_screen == cScreen ) {
            evq.call(showGYRO);
            // showGYRO();
        }
        else if ( HR_screen == cScreen ) {
            evq.call(showHR);
        }


        if ( powerEN ) { // force power off
            evq.break_dispatch();   // stop events; let last one finish
            break;                  // stop main loop
        }
        wait(0.5f);
    }
}




/* implement prototypes */
/**
 * helper function to print label onto OLED from another file
 */
void oled_printLabel(uint8_t *txt, uint8_t xCrd, uint8_t yCrd) {
    oled.Label( txt, xCrd, yCrd );
}

/**
 * helper function to print TextBox onto OLED from another file
 */
void oled_printTextBox(uint8_t *txt, ...) {

}


void systemOFF() {
    powerEN = 1;        // check to turn off system
    oled.DimScreenOFF();
    oled.PowerOFF();
}


void flipLED() {
    led = !led;
}


void btnLeft_helper(void) {
    if ( HTU_screen == cScreen ) cScreen = nScreen - 1;
    else cScreen--;
    startHaptic();
}
/**
 * handles left button press
 */
void btnLeft() {
    evq.call(btnLeft_helper);
    // btnLeft_helper();
}


// void hrBtn_helper() {
//     if ( HR_screen == cScreen ) {
//         hrInter.enable_irq();
//     }
//     else {
//         hrInter.disable_irq();
//     }
// }



void btnRight_helper(void) {
    cScreen++;
    cScreen %= nScreen;
    startHaptic();
}
/**
 * handles right button press
 */
void btnRight() {
    evq.call(btnRight_helper);
    // btnRight_helper();
    startHaptic();
}


/**
 * gets HTU sensor data and print to OLED
 * only if screen active
 */
void showHTU() {
    cTemp = htu.sample_ctemp();
    humid = htu.sample_humid();
    // wait(0.1);
    printf("Temp: %d\r\n", cTemp);
    printf("Humid: %d\r\n\n", humid);
    /* Show Temp */
    txtProps.fontColor = COLOR_RED;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties(&txtProps);
    if ( lScreen != cScreen ) {
        wait_ms(10);
        oled.FillScreen(COLOR_BLACK);
        lScreen = cScreen;
        hrInter.disable_irq();
    }
    strcpy((char *) txt,"Temp.");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy((char *) txt,"dC");
    oled.Label((uint8_t *)txt,82,25);
    sprintf(txt, "%i", cTemp);
    oled.TextBox((uint8_t *)txt, 57, 25, 20, 15);
    /* Show Humidity */
    txtProps.fontColor = COLOR_BLUE;
    oled.SetTextProperties( &txtProps );
    strcpy((char *) txt,"Humd.");
    oled.Label((uint8_t *)txt, 5, 39);
    strcpy((char *) txt,"%");
    oled.Label((uint8_t *)txt,82,39); 
    sprintf(txt, "%i",humid);
    oled.TextBox((uint8_t *)txt, 57, 39, 20, 15);
}


// TODO insert haptics here

/**
 * gets FXOS sensor data and print to OLED
 * only if screen active
 */
void showACCEL() {
    accel.acquire_accel_data_g(accel_data);
    accel_rms = sqrt(((accel_data[0]*accel_data[0])+(accel_data[1]*accel_data[1])+(accel_data[2]*accel_data[2]))/3);
    printf("Accelerometer \tX-Axis %4.2f \tY-Axis %4.2f \tZ-Axis %4.2f \tRMS %4.2f\n\r",
        accel_data[0],
        accel_data[1],
        accel_data[2],
        accel_rms
    );
    txtProps.fontColor = COLOR_MAGENTA;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties(&txtProps);
    if ( lScreen != cScreen ) {
        wait_ms(10);
        oled.FillScreen(COLOR_BLACK); // uncomment if another screen added
        lScreen = cScreen;
        hrInter.disable_irq();
    }
    wait_ms(1);
    strcpy((char *) txt,"Accel. X");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy((char *) txt,"Accel. Y");
    oled.Label((uint8_t *)txt, 5, 39);
    strcpy((char *) txt,"Accel. Z");
    oled.Label((uint8_t *)txt, 5, 53);
    /* x-axis */
    sprintf(txt, "%4.2f", accel_data[0]);
    oled.TextBox((uint8_t *)txt, 50, 25, 40, 15);
    /* y-axis */
    sprintf(txt, "%4.2f",accel_data[1]);
    oled.TextBox((uint8_t *)txt, 50, 39, 40, 15);
    /* z-axis */
    sprintf(txt, "%4.2f",accel_data[2]);
    oled.TextBox((uint8_t *)txt, 50, 53, 40, 15);
    /* Display units on top */
    strcpy(txt, "m/s^2");
    oled.Label((uint8_t *)txt, 60, 11);
}


/**
 * gets FXOS sensor data and print to OLED
 * only if screen active
 */
void showGYRO() {
    gyro.acquire_gyro_data_dps(gyro_data);
    gyro_rms = sqrt(((gyro_data[0]*gyro_data[0])+(gyro_data[1]*gyro_data[1])+(gyro_data[2]*gyro_data[2]))/3);
    printf("Gyroscope \tRoll (G) %4.2f \tPitch (G) %4.2f \tYaw (G) %4.2f \tRMS %4.2f\n\r",
        gyro_data[0],
        gyro_data[1],
        gyro_data[2],
        gyro_rms
    );
    wait_us(10);
    gx = gyro_data[0];
    gy = gyro_data[1];
    gz = gyro_data[2];

    txtProps.fontColor = COLOR_GREEN;
    oled.SetTextProperties( &txtProps );
    if ( lScreen != cScreen ) {
        wait_ms(10);
        oled.FillScreen(COLOR_BLACK);
        lScreen = cScreen;
        hrInter.disable_irq();
    }
    /* Display Legends */
    strcpy((char *) txt,"Roll:");
    oled.Label((uint8_t *)txt,3,25);
    /* Format the value */
    sprintf(txt,"%4.2f",gx);
    /* Display time reading in 35px by 15px textbox */
    oled.TextBox((uint8_t *)txt,50,25,40,15);
    /* Display Legends */
    strcpy((char *) txt,"Pitch:");
    oled.Label((uint8_t *)txt,3,39);
    /* Format the value */
    sprintf(txt,"%4.2f",gy);
    /* Display time reading in 35px by 15px textbox */
    oled.TextBox((uint8_t *)txt,50,39,40,15);
    /* Display Legends */
    // strcpy((char *) txt,"Yaw (dps):");
    strcpy((char *) txt,"Yaw:");
    oled.Label((uint8_t *)txt,3,53);  
    /* Format the value */
    sprintf(txt,"%4.2f",gz);
    /* Display time reading in px by px textbox */
    oled.TextBox((uint8_t *)txt,50,53,40,15);
    /* Display units on top */
    strcpy(txt, "dps");
    oled.Label((uint8_t *)txt, 70, 11);
}


/**
 * gets HR sensor data and print to OLED
 * only if screen is active
 */
void showHR() {
    if ( lScreen != cScreen ) {
        wait_ms(10);
        oled.FillScreen(COLOR_BLACK);
        lScreen = cScreen;
        hrInter.enable_irq();
    }
    readHR();
    printf("HR: %d\r\n", realHeartRate);
    txtProps.fontColor = COLOR_RED;
    oled.SetTextProperties( &txtProps );
    
    strcpy((char *) txt, "Heart Rate:");
    oled.Label((uint8_t *)txt, 3, 39);
    if ( 0 == realHeartRate ) {
        strcpy((char *)txt, "0");
    }
    else {
        sprintf(txt, "%d", realHeartRate);
    }
    oled.TextBox((uint8_t *)txt,70,39,20,15);
}

/**
 * adds HR event to queue
 */
void interHdlr() {
    evq.call(readHR);
}


/**
 * gathers data from heart rate monitor
 */
void readHR() {
    // printf("reading HR\r\n");
    MAX30101::InterruptBitField_u interruptStatus;
    hr.getInterruptStatus(interruptStatus);
    // printf("Interrupt Status: 0x%02x\r\n", interruptStatus.all);
    
    if (interruptStatus.bits.pwr_rdy == 0x1) {
        // printf("Powered on\r\n");
        
        // Soft reset
        MAX30101::ModeConfiguration_u modeConf;
        modeConf.all = 0;
        modeConf.bits.reset = 1;
        hr.setModeConfiguration(modeConf);
        wait(0.01);
        
        // Configure FIFO
        MAX30101::FIFO_Configuration_u fifoConf;
        hr.getFIFOConfiguration(fifoConf);
        // pc.printf("FIFO Configuration: 0x%02x\r\n", fifoConf.all);
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
        // pc.printf("SpO2 Configuration: 0x%02x\r\n", spo2Conf.all);
        
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
            
            // printf("sample: %u\r\n", *(uint32_t *) sample);
            num = *(uint32_t *) sample;
            if (num < 310000){
                    realHeartRate = 0;
                    // printf("keep closer to your hand \r\n");
            }
            else {
                //realHeartRate = 65;
                realHeartRate = (num - 310000)/100;
                if (realHeartRate > 45){
                    // printf("%d\r\n", realHeartRate);
                }
            }
        }
    }
    interruptStatus.all = 0xFF;
    if (mask_ppg == 1) {
        interruptStatus.bits.ppg_rdy = 0;
    }
    hr.enableInterrupts(interruptStatus);
}

/**
 * starts haptic feedback
 */
void startHaptic() {
    // hapticTimer.start(50);
    hapticTimer.attach(&stopHaptic, 0.05f);
    haptic = 1;
}

/**
 * stops haptic feedback
 */
void stopHaptic() {
    // hapticTimer.stop();
    hapticTimer.detach();
    haptic = 0;
}