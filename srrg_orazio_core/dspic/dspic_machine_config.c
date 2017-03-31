#include "dspic_machine_config.h"

// Select external XT oscillator with PLL support
_FOSCSEL(FNOSC_PRIPLL);
// Enable clock switching and configure POSC in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Disabling watchdog
_FWDT(FWDTEN_OFF)
