/*
 * File:   main.c
 * Author: bart
 *
 * Created on April 26, 2016, 11:50 AM
 */


#include "dspic_machine_config.h"
#include "dspic_settings.h"
#include "generic_utils.h"
#include "dspic_serial.h"
#include <string.h>
#include <stdio.h>
#include "Arduino.h"

volatile int timer_count=0;
void __attribute__((__interrupt__, __no_auto_psv__)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    timer_count++;
}

int main(void) {
  //initArduinoEmulator();
    
      Micro_init();
      Timer1_init();
      serialInit();
    

    IEC0bits.U1TXIE = 0;

    int rx_string_length=0;
    char sbuf[128]; 
    sbuf[0]=0;
    while(1){
      int i=0;
      int num_chars=readBufferSize();
      if (num_chars) {
	while(i<num_chars){
	  sbuf[i]=readByte();
	  i++;
	}
      }
      sbuf[i]=0;
      uint8_t obuf[256];
      int obuf_l=sprintf((char*) obuf, "int: %d, [%s]\n", timer_count, (char*) sbuf);
      writeBuffer(obuf, obuf_l);
      flushBuffer();
      DelayN1ms(10);
    }
    
    
}
