#ifndef MPRLS_PRESSURE_H_
#define MPRLS_PRESSURE_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <stdbool.h>
#include <delay.h>
#define MPRLS_I2C_ADDRESS 0x18
// see page 18 of datasheet
#define MPRLS_READ_CMD   \
    {                    \
        0xAA, 0x00, 0x00 \
    }
// see page 14 of datasheet (assuming status reg is identical for I2C and SPI)
#define MPRLS_STATUS_REG_BUSY_MASK (1 << 5)
#define MPRLS_STATUS_REG_POWER_MASK (1 << 6)
#define MPRLS_STATUS_REG_MATH_SATURATION_MASK (1 << 0)
#define MPRLS_STATUS_REG_MEM_INTEGRITY_MASK (1 << 2)
// see page 20 of datasheet
#define MPRLS_OUTMAX 15099494UL
#define MPRLS_OUTMIN 1677722UL
#define MPRLS_PMAX 25
#define MPRLS_PMIN 0
    // function prototypes
    uint32_t MPRLS_meas_StatusReg_polling(uint32_t BMSPI_instance);
    uint32_t MPRLS_meas_waiting(uint32_t BMSPI_instance);
    uint32_t MPRLS_meas_EOC_polling(uint32_t BMI2C_instance, uint8_t EOC_port,
                                    uint8_t EOC_pin);
    void MPRLS_setup_EOC_interrupt(uint32_t BMI2C_instance);
    uint32_t MPRLS_eval_EOC_interrupt(uint32_t BMI2C_instance);
    double MPRLS_binary_to_pressure_psi(uint32_t output);
    double MPRLS_binary_to_pressure_hPa(uint32_t output);
#ifdef __cplusplus
}
#endif
#endif