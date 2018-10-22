#include "teensy_general.h" 
#include "t_usb.h"
#include <stdlib.h>
#include <math.h>

//function definitions
void set_IO(void);
void set_timer(void);
void set_ADC_params(void);
void set_PWM_frequency(void);
void set_ADC_channel(int choice);
int read_ADC(void);
void set_direction_motor1(float error);
void set_direction_motor2(float error);
float PID_motor1(int error);
float PID_motor2(int error);

//declare global variables
int error_sum1 = 0, error_sum2 = 0;

int main(void)
{
    m_usb_init(); //initialize usb subsystem
    teensy_clockdivide(0); //set the clock speed
    teensy_led(OFF);
    
    set_IO(); //set IO pins
    set_timer(); //set timer for PWM
    set_ADC_params(); //set ADC parameters
    set_PWM_frequency(); //set PWM frequency
    
    int m_ADC1, s_ADC1, m_ADC2, s_ADC2, threshold;
    float pidmax, pid1, pid2, dc1, dc2;
    float error1, error2;
    
    pidmax = 0.000977; //1/1023 float value
    threshold = 100; //threshold for error

    for(;;){

        //read from port F0
        set_ADC_channel(0);
        m_ADC1 = read_ADC();
        
        //read from port F1
        set_ADC_channel(1);
        s_ADC1 = read_ADC();
        error1 = m_ADC1 - s_ADC1; //desired-actual
        
        //read from port F4
        set_ADC_channel(4);
        m_ADC2 = read_ADC();
        
        //read from port F5
        set_ADC_channel(5);
        s_ADC2 = read_ADC();
        error2 = m_ADC2 - s_ADC2; //desired-actual
        
        while(abs(error1) > threshold || abs(error2) > threshold) {
            teensy_led(ON);
            
            //position control for motor 1
            if(abs(error1) > threshold) {
                //set direction based on sign of error
                set_direction_motor1(error1); 
                //PID calculation
                pid1 = fabs(PID_motor1(error1)); //take magnitude of pid value
                //set duty cycle of motor PWM
                //constrain pid value
                if(pid1 > 1023)
                    pid1 = 1023.0;
                
                dc1 = pid1*pidmax; //convert to fraction
                
                if(dc1 > 0.1) {
                    OCR1C = OCR1A*(1-dc1); //set PWM 
                }
                else { //stop motor because it wont move for duty cycle < 0.1
                    dc1 = 0.0;
                    OCR1C = OCR1A*(1-dc1); //set PWM  
                }
            }
            else { //stop motor if error < threshold
                dc1 = 0.0;
                OCR1C = OCR1A*(1-dc1); //set PWM  
            }
            
            //reread positions
            //read from port F0
            set_ADC_channel(0);
            m_ADC1 = read_ADC();

            //read from port F1
            set_ADC_channel(1);
            s_ADC1 = read_ADC();
            error1 = m_ADC1 - s_ADC1;  
            
            //position control for motor 2
            if(abs(error2) > threshold) {
                //set direction based on sign of error
                set_direction_motor2(error2);  
                //PID calculation
                pid2 = fabs(PID_motor2(error2)); //take magnitude of pid value
                //set duty cycle of motor PWM
                //constrain pid value
                if(pid2 > 1023)
                    pid2 = 1023.0;
                
                dc2 = pid1*pidmax; //convert to fraction
                
                if(dc2 > 0.1) {
                    OCR3A = ICR3*(1.0-dc2); //set PWM 
                }
                else { //stop motor because it wont move for duty cycle < 0.1
                    //error1 = threshold - 1;
                    dc2 = 0.0;
                    OCR3A = ICR3*(1.0-dc2); //set PWM  
                }
            }
            else { //stop motor if error < threshold
                dc2 = 0.0;
                OCR3A = ICR3*(1.0-dc2); //set PWM  
            }
            
            //reread positions
            //read from port F4
            set_ADC_channel(4);
            m_ADC2 = read_ADC();

            //read from port F5
            set_ADC_channel(5);
            s_ADC2 = read_ADC();
            error2 = m_ADC2 - s_ADC2;
          }
        
        teensy_led(OFF);
        //turn motor off once in position
        dc1 = 0.0; dc2 = 0.0;
        OCR1C = OCR1A*(1-dc1);
        OCR3A = ICR3*(1.0-dc2);
    }

    return 0;
}

void set_IO() {
    clear(DDRF,0);
    clear(PORTF,0); //set input pin as F0
    
    clear(DDRF,1);
    clear(PORTF,1); //set input pin as F1
    
    clear(DDRF,4);
    clear(PORTF,4); //set input pin as F4
    
    clear(DDRF,5);
    clear(PORTF,5); //set input pin as F5
    
    set(DDRB,7);
    set(PORTB,7); //set PWM output for motor 1
    
    set(DDRC,6);
    set(PORTC,6); //set PWM output for motor 2
    
    return;
}

void set_timer() {
    //use timer 1
    set(TCCR1B,CS12); clear(TCCR1B,CS11); set(TCCR1B,CS10); // Prescaler is 1024
    set(TCCR1B,WGM12); set(TCCR1A,WGM10); set(TCCR1B,WGM13); set(TCCR1A,WGM11); //Choose mode 15 up to OCR1A
    set(TCCR1A,COM1C1); set(TCCR1A,COM1C0); // set at OCR1C
    
    //timer 3
    set(TCCR3B,CS32); clear(TCCR3B,CS31); set(TCCR3B,CS30); // Prescaler is 1024
    set(TCCR3B,WGM33); set(TCCR3B,WGM32); set(TCCR3A,WGM31); clear(TCCR3A,WGM30); //Choose mode 14 upto ICR3 PWM mode
    set(TCCR3A,COM3A1); set(TCCR3A,COM3A0); //set at OCR3A, clear at rollover
    
    return;
}

void set_ADC_params() {
    clear(ADMUX,REFS1);
    set(ADMUX,REFS0); //choose voltage reference at VCC
    set(ADCSRA,ADPS2);
    clear(ADCSRA,ADPS1);
    clear(ADCSRA,ADPS0); //set ADC prescaler at 16
    set(DIDR0,ADC0D); //disable digital inputs
    set(DIDR0,ADC1D); //disable digital inputs
    set(DIDR0,ADC4D); //disable digital inputs
    set(DIDR0,ADC5D); //disable digital inputs
    //set(ADCSRA,ADATE); //commented because perform ADC only on demand
    //set(ADCSRA,ADIE); //commented because no interrupts needed
    
    return;
}

void set_PWM_frequency() {
    OCR1A = 156; //100Hz
    ICR3 = 156; //100 Hz
    return;
}

void set_ADC_channel(int choice) {
    
    switch(choice) {
        case 0:
            clear(ADCSRB,MUX5);
            clear(ADMUX,MUX2);
            clear(ADMUX,MUX1);
            clear(ADMUX,MUX0); //hook ADC up to F0
            break;
        case 1:
            clear(ADCSRB,MUX5);
            clear(ADMUX,MUX2);
            clear(ADMUX,MUX1);
            set(ADMUX,MUX0); //hook ADC up to F1 
            break;
        case 4:
            clear(ADCSRB,MUX5);
            set(ADMUX,MUX2);
            clear(ADMUX,MUX1);
            clear(ADMUX,MUX0); //hook ADC up to F4
            break;
        case 5:
            clear(ADCSRB,MUX5);
            set(ADMUX,MUX2);
            clear(ADMUX,MUX1);
            set(ADMUX,MUX0); //hook ADC up to F5
            break;
    }
    
    set(ADCSRA,ADEN); //enable ADC

    return;
}

int read_ADC() {
    set(ADCSRA,ADSC); //start converting
    
    int v;
    while(!bit_is_set(ADCSRA, ADIF)); //wait untill conversion is finished
    v = ADC;
    
    set(ADCSRA, ADIF); //reset flags
    clear(ADCSRA, ADSC);
    return v;
}

void set_direction_motor1(float error) {
    //set direction based on sign of error for motor 1
    if(error > 0) {
        set(DDRB, 5); set(PORTB, 5);
        clear(DDRB, 6); clear(PORTB, 6);
    }
    else if(error < 0) {
        set(DDRB, 6); set(PORTB, 6);
        clear(DDRB, 5); clear(PORTB, 5);
    }
    return;
}

void set_direction_motor2(float error) {
    //set direction based on sign of error for motor 2
    if(error > 0) {
        set(DDRD, 1); set(PORTD, 1);
        clear(DDRD, 2); clear(PORTD, 6);
    }
    else if(error < 0) {
        set(DDRD, 2); set(PORTD, 2);
        clear(DDRD, 1); clear(PORTD, 1);
    }
    return;
}

float PID_motor1(int error) {
    float kp, ki, kd;
    float P, I, D;
    static int error_0;
    
    kp = 5.0; ki = 0.0; kd = 0.0;
    error_sum1 += error;
    
    P = kp*error;
    I = ki*error_sum1;
    D = kd*(error - error_0);
    
    error_0 = error;
    return(P+I+D);
}

float PID_motor2(int error) {
    float kp, ki, kd;
    float P, I, D;
    static int error_0;
    
    kp = 5.0; ki = 0.01; kd = 0.0;
    error_sum2 += error;
    
    P = kp*error;
    I = ki*error_sum1;
    D = kd*(error - error_0);
    
    error_0 = error;
    return(P+I+D);
}