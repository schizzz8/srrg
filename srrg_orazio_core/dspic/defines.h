/* 
 * File:   defines.h
 * Author: malcom
 *
 * Created on June 12, 2013, 12:41 PM
 */

#ifndef DEFINES_H
#define	DEFINES_H

#define FIN 10000000
#define N1 2
#define N2 2
#define M 32
#define FOSC FIN*(M/(N1*N2)) //is in the range of 12.5 MHz to 80 MHz, which generates device operating speeds of 6.25-40 MIPS
#define FCY FOSC/2 // FCY defines the MIPS
#define TMR1_VALUE 40000 //1 millisecond \\________|\ PAY ATTENTION
#define TMR2_VALUE 40000 //1 millisecond //        |/ TO THIS!!!!!!
//UART BAUDRATE
#define BAUDRATE 115200
//#define BAUDRATE 57600
#define BRGVAL (FCY/(4*BAUDRATE))-1

//Led configuration
#define LED1 PORTAbits.RA4
#define LED2 PORTBbits.RB4
#define LED1CONF _TRISA4
#define LED2CONF _TRISB4


#define PWMMODE 0   //pwm 0 TANK 3A
                    //pwm 1 complementary
#endif	/* DEFINES_H */

