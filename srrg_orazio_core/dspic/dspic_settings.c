#include <timer.h>
#include "defines.h"
void Clock_init() {
    // Configure PLL prescaler, PLL postscaler, and PLL divisor
    //http://ww1.microchip.com/downloads/en/DeviceDoc/70291G.pdf page 145
    OSCTUN = 0;
    PLLFBD = 30; // M = 32
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    // Initiate clock switch to primary oscillator with PLL (NOSC = 0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);
    // Wait for clock switch to occur
    while (OSCCONbits.COSC != 0b011);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
}

void Pin_init() {
    //PPSUnLock;
    //PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP3); //UART1TX su RP3
    //PPSInput (PPS_U1RX,PPS_RP2);
    //PPSLock;

    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bclr OSCCON, #6 ");

    //UART
    RPINR18bits.U1RXR = 3; //INPUT
    RPOR1bits.RP2R = 3; //OUTPUT

    //ENCODER
    //***************************
    // Assign QEA1 To Pin RP10
    //***************************
    RPINR14bits.QEA1R = 10;

    //***************************
    // Assign QEB1 To Pin RP11
    //***************************
    RPINR14bits.QEB1R = 11;

    //***************************
    // Assign QEA2 To Pin RP6
    //***************************
    RPINR16bits.QEA2R = 6;

    //***************************
    // Assign QEB2 To Pin RP5
    //***************************
    RPINR16bits.QEB2R = 5;

    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");

    AD1PCFGL = 0xFFFF; //FONDAMENTALE PER LA UART, DEVO CONFIGURARE QUEI PIN COME DIGITALI, DI BASE SONO ANALOGICI
}

void Timer1_init() {
    ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
    WriteTimer1(0);
    OpenTimer1(T1_ON &
            T1_GATE_OFF &
            T1_IDLE_STOP &
            T1_PS_1_1 &
            T1_SYNC_EXT_OFF &
            T1_SOURCE_INT,
            TMR1_VALUE);
}

void Timer2_init() {
    ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
    WriteTimer2(0);
    OpenTimer2(T2_ON &
            T2_GATE_OFF &
            T2_IDLE_STOP &
            T2_PS_1_1 &
            T2_SOURCE_INT,
            TMR2_VALUE);
}


void Led_init() {
    //tristate settings as output
    LED1CONF = 0;
    LED2CONF = 0;
    LED1 = 0;
    LED2 = 0;
}

void Micro_init()
{
    Clock_init();
    Pin_init();
    Timer1_init();
    //Timer2_init();
    //Led_init();
    
   
   
}
