#include "MPRLSpressure.h"
#include "BareMetal_I2C.h"
// Please note: BMI2C must be initialized before calling these functions!
// Method 1: Polling of Status register
uint32_t MPRLS_meas_StatusReg_polling(uint32_t BMI2C_instance) {
// initial setup, see page 18 of MPRLS datasheet
uint8_t tx_buffer[] = MPRLS_READ_CMD;
uint8_t rx_buffer[] = {0,0,0,0};
// trigger conversion with predefined sequence
BMI2C_Write(BMI2C_instance, tx_buffer, 3);
// Continuously read until status reg shows conversion complete
/* TO DO*/
// now read status and result
/* TO DO*/
// Optional To Do: check status? is in rx_buffer[0]...
return (uint32_t) ( (rx_buffer[1]<<16) + (rx_buffer[2]<<8) + (rx_buffer[3]) );
}
// Method 2: Fixed wait time with delay statement (e.g., 10ms)
uint32_t MPRLS_meas_waiting(uint32_t BMI2C_instance) {
/* TO DO */
}
// Method 3.1: EOC pin polling
uint32_t MPRLS_meas_EOC_polling(uint32_t BMI2C_instance, uint8_t EOC_port,
uint8_t EOC_pin) {
/* TO DO*/
}
// This triggers a readout, but does not read the result. Attach an interrupt to the EOC pin in your main file and call MPRLS_eval_EOC_interrupt() there.
// Method 3.2: EOC pin interrupt
void MPRLS_setup_EOC_interrupt(uint32_t BMI2C_instance) {
/* TO DO*/
}

uint32_t MPRLS_eval_EOC_interrupt(uint32_t BMI2C_instance) {
/* TO DO */
}
// see datasheet page 20
double MPRLS_binary_to_pressure_psi(uint32_t output) {
/* TO DO - SEE YOUR PREPARATION WORK*/
}
double MPRLS_binary_to_pressure_hPa(uint32_t output){
double psi = MPRLS_binary_to_pressure_psi(output);
return psi * /* TO DO */; // conversion factor --> online!
}