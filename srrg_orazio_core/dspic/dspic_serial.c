#include "dspic_serial.h"
#include <uart.h>

#define BUFFER_SIZE 256

char tx_buffer[BUFFER_SIZE];
volatile uint8_t tx_start=0;
volatile uint8_t tx_end=0;
volatile uint16_t tx_size=0;


void serialInit() {
    CloseUART1();
    ConfigIntUART1(UART_RX_INT_EN // RX interrupt enable
            & UART_TX_INT_EN // TX interrupt enable
            & UART_RX_INT_PR2 // RX interrupt priority 2
            & UART_TX_INT_PR2); // TX interrupt priority 2
    /* Configuration du registre U1MODE */

    

    unsigned int U1MODEvalue = UART_EN //Bit 15 : Tx, Rx enable
            & UART_IDLE_CON //Bit 13: continue module operation in Idle mode
            & UART_IrDA_DISABLE //Bit 12 : IrDA encoder and decoder disabled
            & UART_MODE_FLOW //Bit 11 : Flow Control mode
            & UART_UEN_00 //Bit 8 & 9: TX,RX enabled, CTS,RTS not
            & UART_EN_WAKE //Bit 7 : UARTx continues to sample the UxRX pin; interrupt generated on falling edge; bit cleared in hardware on following rising edge.
            & UART_DIS_LOOPBACK //Bit 6 : Loopback disabled
            & UART_DIS_ABAUD //Bit 5 : Baud rate measurement disabled or completed
            & UART_UXRX_IDLE_ONE //Bit 4 : Idle state = 1
            & UART_BRGH_FOUR //Bit 3 : 16 clocks per bit period
            & UART_NO_PAR_8BIT //Bit 1 & 2 : 8-bit data, no parity
            & UART_1STOPBIT; //Bit 0 : One Stop Bit

    /* Configuration du registre U1STA */
    unsigned int U1STAvalue =  //Bit 15 & 13 : Interrupt when character is transfered
             UART_INT_TX
            & UART_IrDA_POL_INV_ZERO //Bit 14 : IrDA config
            & UART_SYNC_BREAK_DISABLED //Bit 11 : Sync Break disabled
            & UART_TX_ENABLE //Bit 10 : Transmit enabled
            & UART_TX_BUF_NOT_FUL //Bit 9 : Transmit buffer is not full
            & UART_INT_RX_CHAR //Bit 6 & 7 : Interrupt when any character is received
            & UART_ADR_DETECT_DIS //Bit 5 : Adress Detect mode disabled
            & UART_RX_OVERRUN_CLEAR; //Bit 1 *Read Only Bit* et CLEAR
    
    IEC0bits.U1TXIE = 0; //DISABLE TRANSMIT INTERRUPT
    IEC0bits.U1RXIE = 0; //DISABLE RECEIVE INTERRUPT
    IFS0bits.U1TXIF = 0; //CLEAR TRANSMIT FLAG
    IFS0bits.U1RXIF = 0; //CLEAR RECEIVE INTERRUPT
    OpenUART1(U1MODEvalue, U1STAvalue, BRGVAL);
    IEC0bits.U1TXIE = 0; //DISABLE TRANSMIT INTERRUPT
    IEC0bits.U1RXIE = 1; //ENABLE RECEIVE INTERRUPT
    IFS0bits.U1TXIF = 0; //CLEAR TRANSMIT FLAG
    IFS0bits.U1RXIF = 0; //CLEAR RECEIVE INTERRUPT
}


void __attribute__((__interrupt__, __no_auto_psv__)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;
    if (tx_size){
        U1TXREG=tx_buffer[tx_start];
        tx_start++;
        tx_size--;
    }else{
        IEC0bits.U1TXIE = 0;
    }
}

char writeByte(uint8_t c){
    if (tx_size>=BUFFER_SIZE)
        return 0;
    tx_buffer[tx_end] = c;
    tx_end++;
    tx_size++;
}

char writeBuffer(const uint8_t* b, int n){
    int i=0;
    while(i<n){
      if (writeByte(*b)){
	i++;
	b++;
      } else
	return i;
    }
    return i;
}

int writeBufferSize(){
    return tx_size;
}

void flushBuffer(){
    if (tx_size){
        IEC0bits.U1TXIE = 1;
        U1TXREG=tx_buffer[tx_start];
        tx_start++;
        tx_size--;
    }
}

char rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_start=0;
volatile uint8_t rx_end=0;
volatile uint16_t rx_size=0;

void __attribute__((__interrupt__, __no_auto_psv__)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;
    if (rx_size<BUFFER_SIZE) {
        rx_buffer[rx_end]=U1RXREG;
        rx_end++;
        rx_size++;
    }
}

uint16_t readBufferSize(){
    return rx_size;
}

uint8_t readByte(){
    if (!rx_size)
        return 0;
    uint8_t c=rx_buffer[rx_start];
    rx_start++;
    rx_size--;
    return c;
}

