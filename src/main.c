#include "teensy_general.h" 
#include "t_usb.h"
#include <stdlib.h>

void set_IO() {
    clear(DDRF,0);
    clear(PORTF,0); //set input pin as F0
    
    clear(DDRF,1);
    clear(PORTF,1); //set input pin as F1
    
    set(DDRB, 7);
    set(PORTB, 7); //set PWM output pin as B7
    
    return;
}

void set_timer() {
    //use timer 1
    set(TCCR1B,CS12); clear(TCCR1B,CS11); set(TCCR1B,CS10); // Prescaler is 1024
    set(TCCR1B,WGM12); set(TCCR1A,WGM10); set(TCCR1B,WGM13); set(TCCR1A,WGM11); //Choose mode 15 up to OCR1A
    set(TCCR1A,COM1C1); set(TCCR1A,COM1C0); // set at OCR1C
    return;
}

void set_ADC_params() {
    clear(ADMUX,REFS1);
    set(ADMUX,REFS0); //choose voltage reference at VCC
    set(ADCSRA,ADPS2);
    clear(ADCSRA,ADPS1);
    clear(ADCSRA,ADPS0); //set ADC prescaler at 16
    set(ADCSRA,ADATE); //have the ADC begin a new conversion immediately after finishing a previous conversion
    set(ADCSRA,ADIE); //call an interrupt when each conversion is finished
    set(ADCSRA,ADEN); //enable ADC
    
    return;
}

void set_ADC_channel(int choice) {
    if(choice == 0) {
        clear(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX0); //hook ADC up to F0
    }
    else {
        clear(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX0); //hook ADC up to F1
    }
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

void set_direction(float error) {
    //set direction based on sign of error
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

int main(void)
{
    m_usb_init(); //initialize usb subsystem
    teensy_clockdivide(0); //set the clock speed
    
    set_IO(); //set IO pins
    set_timer(); //set timer for PWM
    set_ADC_params(); //set ADC parameters
    
    OCR1A = 156; //100Hz
    
    int m_ADC, s_ADC, threshold;
    float pimax, dc;
    float error, kp, ki, P, I, error_sum, pi;
    
    kp = 1.0; ki = 0.01;
    pimax = 0.000977; //1/1023 float value
    threshold = 100;

    for(;;){
        //read from port F0
        set_ADC_channel(0);
        m_ADC = read_ADC();
        
        //read from port F1
        set_ADC_channel(1);
        s_ADC = read_ADC();
        error = m_ADC - s_ADC; //desired-actual
        
        teensy_led(OFF);
        while(abs(error) > threshold) {
            teensy_led(ON); //light up whenever in feedback loop
            
            //PI calculation
            error_sum += error;
            P = kp*error;
            I = ki*error_sum;
            pi = P + I;
            
            set_direction(error); //set direction based on sign of error
            
            //set duty cycle of motor PWM
            //constrain pi value
            if(pi > 1023)
                pi = 1023.0;
            if(pi < 0)
                pi = 0.0;
            
            dc = pi*pimax; //convert to fraction
            OCR1C = OCR1A*(1-dc); //set PWM
            
            if(dc < 0.1) {
                error = threshold + 1; //exit if dc < 0.1 because motor wont move 
            }
            else {
                //feedback from port F1, slave
                s_ADC = read_ADC();
                error = m_ADC - s_ADC;   
            }
        }
        //turn motor off once in position
        dc = 0.0;
        OCR1C = OCR1A*(1-dc);
    }
    return 0;
}

ISR(ADC_vect) {
}