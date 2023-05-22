#include <Arduino.h>

#include "main.h"

BMI2C_init_IOs(BMI2C_INSTANCE_TWI0, BMI2C_PSEL_SCL_REG, 0, MPRLS_SCL);
BMI2C_init_IOs(BMI2C_INSTANCE_TWI0, BMI2C_PSEL_SDA_REG, 0, MPRLS_SDA);
// set address (cf. datasheet of MPRLS) and slowest frequency
BMI2C_set_Address(BMI2C_INSTANCE_TWI0, MPRLS_I2C_ADDRESS);
BMI2C_set_Frequency(BMI2C_INSTANCE_TWI0, BMI2C_FREQ_100KBPS);
// good to go!
BMI2C_enable(BMI2C_INSTANCE_TWI0);

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}