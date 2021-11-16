/**
 * @file main.cpp
 * @author Andrew W
 * @date created 11/04/21
 *
 * @brief implements Hexiwear sensors in an event driven manner
 *
 * last updated: 11/16/21
 */

#include "mbed.h"
// important sensor & other relevant drivers
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_KW40Z.h"
#include "HTU21D.h"
#include "FXOS8700.h"
#include "FXAS21002.h"
#include "MAX30101.h"
// import custom header files
#include "main.h"
#include "images.h"
// import std libraries
#include <string>


/* instantiate sensors and enables */
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);
KW40Z kw40z(PTE24, PTE25);
HTU21D htu(PTB1,PTB0);
FXOS8700 accel(PTC11, PTC10);
FXOS8700 mag(PTC11, PTC10);
FXAS21002 gyro(PTC11,PTC10);
I2C i2cBus(PTB1, PTB0); // helps control hr
MAX30101 hr(i2cBus);
/* Pin Control */
DigitalOut powerEN (PTB12); // Power Enable HTU21D Sensor
DigitalOut led(LED1);       // red LED
DigitalOut pwr1v8(PTA29);
DigitalOut pwr3v3(PTC13);
DigitalOut pwr15v(PTB12);
DigitalOut haptic(PTB9);    // vibration
/* Interrupt Driven */
InterruptIn hrInter(PTB18);
Ticker hapticTimer;


/* create event items */
EventQueue evq(32 * EVENTS_EVENT_SIZE);

/* create threads */
Thread the;
Mutex mut("Controller");

/** declare variables */

// used for printing to OLED
char txt[20];
oled_text_properties_t txtProps = { 0 };
const int8_t nScreen = 5; // number of screens
int8_t cScreen; // current screen
int8_t lScreen; // last screen

// for HTU
int cTemp,
    humid;
// for FXOS
float accel_data[3];
float accel_rms=0.0;
float mag_data[3];  
float mag_rms=0.0;
// for FXAS
float gyro_data[3];
float gyro_rms=0.0;

// for HR
int realHeartRate;
int mask_ppg = 0;
uint32_t count = 0;
uint32_t num;


/* implementation */

int main()
{
    setup();
    splashScreen();

    oled.FillScreen(COLOR_BLACK);
    wait_ms(5);

    the.start(callback(&evq, &EventQueue::dispatch_forever));

    while (true) {
        // evq.call(&getSensors);
        evq.call(&updateScreen);
        wait_ms(500);
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
    // TODO
}


void oled_reset() {
    oled.FillScreen(COLOR_BLACK);
}


/**
 * setup everything for system
 */
void setup() {
    /** setup sensors */
    led = 1;
    powerEN = 0;
    pwr1v8 = 1;
    pwr3v3 = 1;
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
    hrInter.fall(interHdlr);
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

    /* setup next screen */
    cScreen = ACCEL_screen;
    lScreen = cScreen;
    
    oled.GetTextProperties( &txtProps );
}

/**
 * shows opening logos on startup
 */
void splashScreen() {
    /* opening spash screens */
    const uint8_t *logo_img1 = mrehablogo_bmp;
    const uint8_t *logo_img2 = hexiwear_logo_bmp;
    oled.DrawImage( logo_img1, 0, 0 );
    wait(2.0f); // wait 2s
    oled.DrawImage( logo_img2, 0, 0 );
    wait(3.0f); // wait 3s
}


void systemOFF() {
    powerEN = 1;        // check to turn off system
    oled.DimScreenOFF();
    oled.PowerOFF();
}


void flipLED() {
    led = !led;
}

/**
 * handles left button press as scheduled
 */
static void btnLeft_helper(void) {
    if ( HTU_screen == cScreen ) cScreen = nScreen - 1;
    else cScreen--;
    startHaptic();
}
/**
 * handles left button press
 */
void btnLeft() {
    evq.call(btnLeft_helper);
}

/**
 * handles right button press as scheduled
 */
static void btnRight_helper(void) {
    cScreen++;
    cScreen %= nScreen;
    startHaptic();
}
/**
 * handles right button press
 */
void btnRight() {
    evq.call(btnRight_helper);
    startHaptic();
}


/**
 * produces OLED display
 * currently only shows sensor results
 */
void updateScreen() {
    showSensor();
}

/**
 * gets the current data from each sensor
 */
void getSensors() {
    // evq.call(&readHTU);
    // evq.call(&readACCEL);
    // evq.call(&readMAG);
    // evq.call(&readGYRO);
    // evq.call(&readHR);
    mut.lock();
    readHTU();
    readACCEL();
    readMAG();
    readGYRO();
    // readHR();
    mut.unlock();
}

/**
 * shows current screen data
 * also gathers necessary data
 */
void showSensor() {
    if ( lScreen != cScreen ) { // reset if screen changes
        oled.FillScreen(COLOR_BLACK);
        hrInter.disable_irq();
        wait_us(5);
        lScreen = cScreen;
    }
    // choose which screen to display
    switch (cScreen) {
        case HTU_screen: {
            /* occasional thread issue if left alone */
            mut.lock();
            readHTU();
            mut.unlock();
            mut.lock();
            showHTU();
            mut.lock();
            break;
        }    
        case ACCEL_screen: {
            mut.lock();
            readACCEL();
            mut.unlock();
            mut.lock();
            showACCEL();
            mut.unlock();
            break;
        }
        case MAG_screen: {
            mut.lock();
            readMAG();
            mut.unlock();
            mut.lock();
            showMAG();
            mut.unlock();
            break;
        }
        case GYRO_screen: {
            mut.lock();
            readGYRO();
            mut.unlock();
            mut.lock();
            showGYRO();
            mut.unlock();
            break;
        }
        case HR_screen: {
            hrInter.enable_irq();
            mut.lock();
            readHR();
            mut.unlock();
            mut.lock();
            showHR();
            mut.unlock();
            break;
        }
        default: {}
    }
}


/**
 * displays HTU results on the OLED
 */
void showHTU() {
    /* Show Temp */
    txtProps.fontColor = COLOR_RED;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties(&txtProps);
    // display text and results
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
    sprintf(txt, "%i", humid);
    oled.TextBox((uint8_t *)txt, 57, 39, 20, 15);
}

/**
 * displays accelerometer results on the OLED
 */
void showACCEL() {
    txtProps.fontColor = COLOR_MAGENTA;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties(&txtProps);
    wait_us(5);
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
    strcpy(txt, "g");
    oled.Label((uint8_t *)txt, 80, 10);
}

/**
 * displays magnetometer results on the OLED
 */
void showMAG() {
    txtProps.fontColor = COLOR_YELLOW;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties(&txtProps);
    wait_us(5);
    strcpy((char *) txt,"Mag. X");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy((char *) txt,"Mag. Y");
    oled.Label((uint8_t *)txt, 5, 39);
    strcpy((char *) txt,"Mag. Z");
    oled.Label((uint8_t *)txt, 5, 53);
    /* x-axis */
    sprintf(txt, "%4.2f", mag_data[0]);
    oled.TextBox((uint8_t *)txt, 50, 25, 40, 15);
    /* y-axis */
    sprintf(txt, "%4.2f", mag_data[1]);
    oled.TextBox((uint8_t *)txt, 50, 39, 40, 15);
    /* z-axis */
    sprintf(txt, "%4.2f", mag_data[2]);
    oled.TextBox((uint8_t *)txt, 50, 53, 40, 15);
    /* Display units on top */
    strcpy(txt, "uT");
    oled.Label((uint8_t *)txt, 70, 11);
}

/**
 * displays gyroscope results on the OLED
 */
void showGYRO() {
    txtProps.fontColor = COLOR_GREEN;
    oled.SetTextProperties( &txtProps );
    /* Display Legends */
    strcpy((char *) txt,"Roll:");
    oled.Label((uint8_t *)txt,3,25);
    /* Format the value */
    sprintf(txt,"%4.2f", gyro_data[0]);
    /* Display time reading in 35px by 15px textbox */
    oled.TextBox((uint8_t *)txt,50,25,40,15);
    /* Display Legends */
    strcpy((char *) txt,"Pitch:");
    oled.Label((uint8_t *)txt,3,39);
    /* Format the value */
    sprintf(txt,"%4.2f", gyro_data[1]);
    /* Display time reading in 35px by 15px textbox */
    oled.TextBox((uint8_t *)txt,50,39,40,15);
    /* Display Legends */
    // strcpy((char *) txt,"Yaw (dps):");
    strcpy((char *) txt,"Yaw:");
    oled.Label((uint8_t *)txt,3,53);  
    /* Format the value */
    sprintf(txt,"%4.2f", gyro_data[2]);
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
    txtProps.fontColor = COLOR_RED;
    oled.SetTextProperties( &txtProps );
    // display text and results
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
 * gets HTU sensor data and print to OLED
 * only if screen active
 */
void readHTU() {
    cTemp = htu.sample_ctemp();
    humid = htu.sample_humid();
    wait_us(5);
    #ifdef DEBUG
    printf("Temp: %d\r\n", cTemp);
    printf("Humid: %d\r\n\n", humid);
    #endif //DEBUG
}

/**
 * gets FXOS accelerometer sensor data and print to OLED
 * only if screen active
 */
void readACCEL() {
    accel.acquire_accel_data_g(accel_data);
    accel_rms = sqrt(((accel_data[0]*accel_data[0])+(accel_data[1]*accel_data[1])+(accel_data[2]*accel_data[2]))/3);
    #ifdef DEBUG
    printf("Accelerometer \tX-Axis %4.2f \tY-Axis %4.2f \tZ-Axis %4.2f \tRMS %4.2f\n\r",
        accel_data[0],
        accel_data[1],
        accel_data[2],
        accel_rms
    );
    #endif // DEBUG
}

/**
 * get FXOS magnometer sensor data and print to OLED
 * only if screen is active
 */
void readMAG() {
    mag.acquire_mag_data_uT(mag_data);
    mag_rms = sqrt(((mag_data[0]*mag_data[0])+(mag_data[1]*mag_data[1])+(mag_data[2]*mag_data[2]))/3);
    #ifdef DEBUG
    printf("Magnetometer \tX-Axis %4.2f \tY-Axis %4.2f \tZ-Axis %4.2f \tRMS %4.2f\n\r",
        mag_data[0],
        mag_data[1],
        mag_data[2],
        mag_rms
    );
    #endif // DEBUG
}

/**
 * gets FXOS sensor data and print to OLED
 * only if screen active
 */
void readGYRO() {
    gyro.acquire_gyro_data_dps(gyro_data);
    gyro_rms = sqrt(((gyro_data[0]*gyro_data[0])+(gyro_data[1]*gyro_data[1])+(gyro_data[2]*gyro_data[2]))/3);
    #ifdef DEBUG
    printf("Gyroscope \tRoll %4.2f \tPitch %4.2f \tYaw %4.2f \tRMS %4.2f\n\r",
        gyro_data[0],
        gyro_data[1],
        gyro_data[2],
        gyro_rms
    );
    #endif // DEBUG
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


/**
 * starts haptic feedback
 */
void startHaptic() {
    hapticTimer.attach(&stopHaptic, 0.05f);
    haptic = 1;
}

/**
 * stops haptic feedback
 */
void stopHaptic() {
    hapticTimer.detach();
    haptic = 0;
}
