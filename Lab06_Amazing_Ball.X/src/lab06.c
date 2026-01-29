#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

#define TOUCH_DIM_X 0
#define TOUCH_DIM_Y 1

#define BW_TS 0.04
#define BW_FC 3.0
#define BW_PI 3.142
#define BW_A ((2.0 * BW_PI * BW_FC * BW_TS) / (1.0 + (2.0 * BW_PI * BW_FC * BW_TS))) //filter coefficient

#define PD_TS 0.04
#define KP_X 0.4
#define KD_X 0.7
#define KP_Y 0.4
#define KD_Y 0.7

#define SP_X 512.0
#define SP_Y 512.0
#define SP_R 100.0 // radius 
#define SP_FREQ_HZ 0.40// circle frequency

#define DUTY_X_FLAT 1450 //LCD to sticker
#define DUTY_Y_FLAT 1400 // sticker to bottom

#define U_TO_DUTY 1.0

uint16_t x, y;
uint16_t x_raw = 0;
uint16_t y_raw = 0;
uint8_t  new_xy_ready = 0;
uint16_t miss_deadline = 0;
uint8_t  exec_busy = 0;
/*
 * Timer Code
 */
void timer_initialize(void)
{
    //configure timer 2 for pwm (10ms period)
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    T2CONbits.TCKPS = TCKPS_64;
    TMR2 = 0;
    PR2 = 4000;
    
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP = 1;
    IEC0bits.T2IE = 1;
}


/*
 * Servo Code
 */
void servo_initialize(void){

    timer_initialize();
    //configure output compare 7 (servo y)and  8(servo X)
    CLEARBIT(TRISDbits.TRISD6);
    CLEARBIT(TRISDbits.TRISD7);

    OC7R = 0;
    OC7RS = 0;
    OC8R = 0;
    OC8RS = 0;
    //SET PWM, no fault, timer2
    OC7CON= 0x0006;
    OC8CON= 0x0006;
            // SET TIMER 2
    SETBIT(T2CONbits.TON);
}

void servo_setduty(uint8_t servo, uint16_t duty_us){
    if (duty_us < 1000) duty_us = 1000;
    if (duty_us > 2000) duty_us = 2000;
    
    uint16_t pulseTicks = duty_us / 5; //5us per tick (64/Fcy)
    
    uint16_t invertedTicks = 4000 - pulseTicks; //20ms/5us = period ticks
    if (servo == 0) OC7RS = invertedTicks;
    else if (servo == 1) OC8RS = invertedTicks;
   
}

/*
 * Touch screen code
 */
void touchscreen_initialize(void)
{
    // Control pins E1,E2,E3 as outputs
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);

    // Safe state
    CLEARBIT(PORTEbits.RE1);
    Nop();
    SETBIT(PORTEbits.RE2);
    Nop();
    SETBIT(PORTEbits.RE3);

    // ADC off during config
    CLEARBIT(AD1CON1bits.ADON);

    //Set input pins
    SETBIT(TRISBbits.TRISB15);
    SETBIT(TRISBbits.TRISB9);
    
    // AN15 (X) and AN9 (Y) as analog
    CLEARBIT(AD1PCFGLbits.PCFG15);
    CLEARBIT(AD1PCFGLbits.PCFG9);
    
    // ADC basic config
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7;
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;

    // ADC on
    SETBIT(AD1CON1bits.ADON);
}

void touchscreen_set_dimension(uint8_t dim)
{
    CLEARBIT(AD1CON1bits.ADON);

    if (dim == TOUCH_DIM_X)
    {
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();

        // Select AN15 for X read
        AD1CHS0bits.CH0SA = 15;
    }
    else if (dim == TOUCH_DIM_Y)
    {
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();

        // Select AN9 for Y read
        AD1CHS0bits.CH0SA = 9;
    }
    
    else
    {
        SETBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();
    }
    
    SETBIT(AD1CON1bits.ADON);
}

uint16_t touchscreen_read(void)
{
    SETBIT(AD1CON1bits.SAMP);
    while (!AD1CON1bits.DONE);
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0;
}

/*
 * PD Controller
 */
float pd_controller_x(float setpoint, float measurement)
{
    static float e_prev_x = 0.0;
    float e = setpoint - measurement;
    float de_dt = 0.0;

    de_dt = (e - e_prev_x) / PD_TS;
    e_prev_x = e;

    return (KP_X * e) + (KD_X * de_dt);
}

float pd_controller_y(float setpoint, float measurement)
{
    static float e_prev_y = 0.0;
    float e = setpoint - measurement;
    float de_dt = 0.0;

    de_dt = (e - e_prev_y) / PD_TS;
    e_prev_y = e;

    return (KP_Y * e) + (KD_Y * de_dt);
}


/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */
float butterworth_filter_x(float x)
{
    static float y_prev_x = 0.0;
    y_prev_x = y_prev_x + BW_A * (x - y_prev_x);
    return y_prev_x;
}

float butterworth_filter_y(float y)
{
    static float y_prev_y = 0.0;
    y_prev_y = y_prev_y + BW_A * (y - y_prev_y);
    return y_prev_y;
}

void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
    uint8_t static state = 0;
    IFS0bits.T2IF = 0;

    if (exec_busy) miss_deadline++;

    if (state == 0) {
        touchscreen_set_dimension(TOUCH_DIM_X);
        state = 1;
    } else if (state == 1) {
        x_raw = touchscreen_read();
        touchscreen_set_dimension(TOUCH_DIM_Y);
        state = 2;
    } else {
        y_raw = touchscreen_read();
        touchscreen_set_dimension(TOUCH_DIM_X);
        state = 1;
        new_xy_ready = 1;
    }
}

void setpoint_circle_step(float *spx, float *spy)
{
    static float t = 0.0;

    const float w = 2*BW_PI * SP_FREQ_HZ;
    *spx = SP_X + (SP_R * cosf(w * t));
    *spy = SP_Y + (SP_R * sinf(w * t));

    t += PD_TS;
}

/*
 * main loop
 */
void main_loop(void)
{
    lcd_clear();
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: 5");
    lcd_locate(0, 2);

    servo_initialize();
    touchscreen_initialize();

    // Set initial position
    servo_setduty(0, DUTY_Y_FLAT);
    servo_setduty(1, DUTY_X_FLAT);

    float x_f = 0.0, y_f = 0.0;
    float ux = 0.0, uy = 0.0;
    uint16_t lcd_div = 0;

    while (TRUE)
    {
        if (!new_xy_ready) continue;

        exec_busy = 1;
        new_xy_ready = 0;

        x_f = butterworth_filter_x((float)x_raw);
        y_f = butterworth_filter_y((float)y_raw);
        
        float spx, spy;
        setpoint_circle_step(&spx, &spy);

        ux = pd_controller_x(spx, x_f);
        uy = pd_controller_y(spy, y_f);

        int dutyX = (int)(DUTY_X_FLAT + (U_TO_DUTY * ux));
        int dutyY = (int)(DUTY_Y_FLAT + (U_TO_DUTY * uy));

        if (dutyX < PWM_MIN_US) dutyX = PWM_MIN_US;
        if (dutyX > PWM_MAX_US) dutyX = PWM_MAX_US;
        if (dutyY < PWM_MIN_US) dutyY = PWM_MIN_US;
        if (dutyY > PWM_MAX_US) dutyY = PWM_MAX_US;

        servo_setduty(1, (uint16_t)dutyX);
        servo_setduty(0, (uint16_t)dutyY);

        lcd_div++;
        if (lcd_div >= 10) {
            lcd_div = 0;
            lcd_locate(0, 6);
            lcd_printf("deadlines missed=%u     ", miss_deadline);
        }

        exec_busy = 0;
    }
}

