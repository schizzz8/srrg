#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <p33FJ128MC802.h>
#include <qei.h>

#ifdef __cplusplus
} // extern "C";
#endif // __cplusplus

Encoder::Encoder() {
  _position_register=0;
  clearCounters();
}

Encoder encoders[num_motors];

void initEncoders(){
  //ENCODER 1
  QEI1CONbits.QEIM = 0b111; //QEI_MODE_x2_MATCH
  QEI1CONbits.SWPAB = 0; //	QEI_INPUTS_SWAP
  QEI1CONbits.QEISIDL = 1; //	QEI_IDLE_STOP
  QEI1CONbits.POSRES = 0; //	QEI_INDEX_RESET_DISABLE
  QEI1CONbits.PCDOUT = 0; //	QEI_NORMAL_IO
  QEI1CONbits.POSRES = 0; //	POS_CNT_ERR_INT_DISABLE

  DFLT1CONbits.QECK = 0; //	QEI_QE_CLK_DIVIDE_1_128
  DFLT1CONbits.QEOUT = 1; //	QEI_QE_OUT_ENABLE

  MAX1CNT = 0xffff;
  POS1CNT = 0;
  ConfigIntQEI1(QEI_INT_ENABLE & QEI_INT_PRI_1);

  //ENCODER 2
  QEI2CONbits.QEIM = 0b111; //	QEI_MODE_x2_MATCH
  QEI2CONbits.SWPAB = 1; //	QEI_INPUTS_SWAP
  QEI2CONbits.QEISIDL = 1; //	QEI_IDLE_STOP
  QEI2CONbits.POSRES = 0; //	QEI_INDEX_RESET_DISABLE
  QEI2CONbits.PCDOUT = 0; //	QEI_NORMAL_IO
  QEI2CONbits.POSRES = 0; //	POS_CNT_ERR_INT_DISABLE

  DFLT2CONbits.QECK = 0; //	QEI_QE_CLK_DIVIDE_1_128
  DFLT2CONbits.QEOUT = 1; //	QEI_QE_OUT_ENABLE

  MAX2CNT = 0xffff;
  POS2CNT = 0;
  ConfigIntQEI2(QEI_INT_ENABLE & QEI_INT_PRI_1);

  encoders[0].setPositionRegister((uint16_t*) 0x01E4);
  encoders[1].setPositionRegister((uint16_t*) 0x01F4);
}
