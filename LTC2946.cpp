/*!
Revised Version
7/4/2020, J. Krause

Reformatted code into a single class, capable of functioning with Teensy 3.6.

-Has full functionality for continuous read of VIN, Current and Power.
-Has limited functionality for Snapshot mode (VIN, Current) (to be done in future if time permits)
-Incorporated experimentally determined constants (one constant to correlate measured with actual value).
-No limit functionality at this time.
-I2C error transaction checking via a single function to check and clear past errors (use in an if statement)
*/

/*!
LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor
The LTC®2946 is a rail-to-rail system monitor that measures
current, voltage, power, charge and energy. It features an
operating range of 2.7V to 100V and includes a shunt regulator
for supplies above 100V. The current measurement common mode
range of 0V to 100V is independent of the input supply.
A 12-bit ADC measures load current, input voltage and an
auxiliary external voltage. Load current and internally
calculated power are integrated over an external clock or
crystal or internal oscillator time base for charge and energy.
An accurate time base allows the LTC2946 to provide measurement
accuracy of better than ±0.6% for charge and ±1% for power and
energy. Minimum and maximum values are stored and an overrange
alert with programmable thresholds minimizes the need for software
polling. Data is reported via a standard I2C interface.
Shutdown mode reduces power consumption to 15uA.
*/


#include <Arduino.h>
#include <stdint.h>
#include "LTC2946.h"
#include <i2c_t3.h>
//#include <Wire.h>

LTC2946::LTC2946(uint8_t wire_num,uint8_t wire_addr) //!constructor
{
    I2C_WIRE = wire_num;
    I2C_ADDRESS = wire_addr;
}

void LTC2946::Setup()
{
    if(I2C_WIRE == 0){
        Wire.begin();
    }else if(I2C_WIRE == 1){
        Wire1.begin();
    }else if(I2C_WIRE == 2){
        Wire2.begin();
    }else if(I2C_WIRE == 3){
        Wire3.begin();
    }
}

bool LTC2946::ErrorCheck()
//Check for past errors in I2C handshake. Resets tracking variable upon read.
{
    if(I2C_ACK == 0){
        return(true);
    }else{
        I2C_ACK = 0;
        return(false);
    }
}

//! Set the constants for converting RAW to values
void LTC2946::SetVINConst(float vin_const){VIN_CONST = vin_const;}
void LTC2946::SetAmperageConst(float i_const){CURRENT_CONST = i_const;}
void LTC2946::SetPowerConst(float w_const){POWER_CONST = w_const;}

void LTC2946::SetContinuous()
// Set default LTC2946 values for Continuous capture mode
{
    int8_t ack = 0;

    LTC2946_mode = 0;
    ack = LTC2946_write(LTC2946_CTRLA_REG, CTRLA);

    I2C_ACK |= ack;
}

void LTC2946::SetSnapShot()
{
    LTC2946_mode = 1;
}

void LTC2946::EnableConversion(bool state)
{
    use_conversion = state;
}

void LTC2946::EnableLegacy(bool state)
{
    use_legacy = state;
}

float LTC2946::ReadVIN()
{
    int8_t ack = 0;
    uint16_t VIN_code;
    float VIN_Return = 0;

    //Continuous Request
    if(LTC2946_mode == 0)
    {
        ack |= LTC2946_read_12_bits(LTC2946_VIN_MSB_REG, &VIN_code);
    }
    //Snapshot Request
    else if(LTC2946_mode == 1)
    {
        int8_t LTC2946_mode;
        LTC2946_mode = LTC2946_CHANNEL_CONFIG_SNAPSHOT | LTC2946_VDD;
        ack |= LTC2946_write(LTC2946_CTRLA_REG, LTC2946_mode);

        uint8_t busy;
        do
        {
            ack |= LTC2946_read(LTC2946_STATUS2_REG, &busy);
        }
        while (0x8 & busy);

        ack |= LTC2946_read_12_bits(LTC2946_VIN_MSB_REG, &VIN_code);
    }

    //Conversion
    if(use_conversion)
    {
        if(use_legacy)
        {
            //Legacy conversion
            VIN_Return = LTC2946_VIN_code_to_voltage(VIN_code);
        }
        else
        {
            //Experimental conversion
            VIN_Return = (float)VIN_code*VIN_CONST;
        }
    }
    else
    {
        //Return RAW value
        VIN_Return = (float)VIN_code;
    }

    //update error
    I2C_ACK |= ack;

    return(VIN_Return);
}

float LTC2946::ReadCurrent()
{
    int8_t ack = 0;
    uint16_t current_code;
    float current_Return = 0;

    //Continuous Request
    if(LTC2946_mode == 0)
    {
        ack |= LTC2946_read_12_bits(LTC2946_DELTA_SENSE_MSB_REG, &current_code);
    }
    //Snapshot Request
    else if(LTC2946_mode == 1)
    {
        int8_t LTC2946_mode;
        LTC2946_mode = LTC2946_CHANNEL_CONFIG_SNAPSHOT | LTC2946_DELTA_SENSE;
        ack |= LTC2946_write(LTC2946_CTRLA_REG, LTC2946_mode);

        uint8_t busy;
        do
        {
            ack |= LTC2946_read(LTC2946_STATUS2_REG, &busy);        //!< Check to see if conversion is still in process
        }
        while (0x8 & busy);

        ack |= LTC2946_read_12_bits(LTC2946_DELTA_SENSE_MSB_REG, &current_code);
    }

    //Conversion
    if(use_conversion)
    {
        if(use_legacy)
        {
            //Legacy conversion
            current_Return = LTC2946_code_to_current(current_code);
        }
        else
        {
            //Experimental conversion
            current_Return = (float)current_code*CURRENT_CONST;
        }
    }
    else
    {
        //Return RAW value
        current_Return = (float)current_code;
    }

    //update error
    I2C_ACK |= ack;

    return(current_Return);
}

float LTC2946::ReadPower()
{
    int8_t ack = 0;
    uint32_t power_code;
    float power_Return = 0;

    //Continuous Request
    if(LTC2946_mode == 0)
    {
        ack |= LTC2946_read_24_bits(LTC2946_POWER_MSB2_REG, &power_code);
    }
    //Snapshot Request
    else if(LTC2946_mode == 1)
    {
        //Not available Yet
    }

    //Conversion
    if(use_conversion)
    {
        if(use_legacy)
        {
            //Legacy conversion
            //Not available yet
        }
        else
        {
            //Experimental conversion
            power_Return = (float)power_code*POWER_CONST;
        }
    }
    else
    {
        //Return RAW value
        power_Return = (float)power_code;
    }

    //update error
    I2C_ACK |= ack;

    return(power_Return);
}




/*
Here is where I would add provisions for limits and colombs and joules, if I ever find time for that
*/






// Write an 8-bit code to the LTC2946.
int8_t LTC2946::LTC2946_write(uint8_t adc_command, uint8_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

//  ack = i2c_write_byte_data(i2c_address, adc_command, code);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        Wire.write(code);
        ack = Wire.endTransmission(false);
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        Wire1.write(code);
        ack = Wire1.endTransmission(false);
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        Wire2.write(code);
        ack = Wire2.endTransmission(false);
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        Wire3.write(code);
        ack = Wire3.endTransmission(false);
    }

    return ack;

}

// Write a 16-bit code to the LTC2946.
int8_t LTC2946::LTC2946_write_16_bits(uint8_t adc_command, uint16_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t ack;

  union
  {
    uint8_t b[2];
    uint16_t w;
  } data;

  data.w = code;

//  ack = i2c_write_word_data(i2c_address, adc_command, code);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        Wire.write(data.b[1]);
        Wire.write(data.b[0]);
        ack = Wire.endTransmission(false);
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        Wire1.write(data.b[1]);
        Wire1.write(data.b[0]);
        ack = Wire1.endTransmission(false);
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        Wire2.write(data.b[1]);
        Wire2.write(data.b[0]);
        ack = Wire2.endTransmission(false);
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        Wire3.write(data.b[1]);
        Wire3.write(data.b[0]);
        ack = Wire3.endTransmission(false);
    }

  return(ack);
}

// Write a 24-bit code to the LTC2946.
int8_t LTC2946::LTC2946_write_24_bits(uint8_t adc_command, uint32_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    union
    {
        int32_t MY_int32;    //!< 32-bit signed integer to be converted to four bytes
        uint32_t MY_uint32;  //!< 32-bit unsigned integer to be converted to four bytes
        uint8_t MY_byte[4];  //!< 4 bytes (unsigned 8-bit integers) to be converted to a 32-bit signed or unsigned integer
    } data;


    data.MY_int32 = code;

    //  ack = i2c_write_block_data(i2c_address, adc_command, (uint8_t) 3, data.LT_byte);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        Wire.write(data.MY_byte[2]);
        Wire.write(data.MY_byte[1]);
        Wire.write(data.MY_byte[0]);
        ack = Wire.endTransmission(false);
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        Wire1.write(data.MY_byte[2]);
        Wire1.write(data.MY_byte[1]);
        Wire1.write(data.MY_byte[0]);
        ack = Wire1.endTransmission(false);
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        Wire2.write(data.MY_byte[2]);
        Wire2.write(data.MY_byte[1]);
        Wire2.write(data.MY_byte[0]);
        ack = Wire2.endTransmission(false);
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        Wire3.write(data.MY_byte[2]);
        Wire3.write(data.MY_byte[1]);
        Wire3.write(data.MY_byte[0]);
        ack = Wire3.endTransmission(false);
    }

    return(ack);
}

int8_t LTC2946::LTC2946_write_32_bits(uint8_t adc_command, uint32_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    union
    {
        int32_t MY_int32;    //!< 32-bit signed integer to be converted to four bytes
        uint32_t MY_uint32;  //!< 32-bit unsigned integer to be converted to four bytes
        uint8_t MY_byte[4];  //!< 4 bytes (unsigned 8-bit integers) to be converted to a 32-bit signed or unsigned integer
    } data;


    data.MY_int32 = code;

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        Wire.write(data.MY_byte[3]);
        Wire.write(data.MY_byte[2]);
        Wire.write(data.MY_byte[1]);
        Wire.write(data.MY_byte[0]);
        ack = Wire.endTransmission(false);
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        Wire1.write(data.MY_byte[3]);
        Wire1.write(data.MY_byte[2]);
        Wire1.write(data.MY_byte[1]);
        Wire1.write(data.MY_byte[0]);
        ack = Wire1.endTransmission(false);
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        Wire2.write(data.MY_byte[3]);
        Wire2.write(data.MY_byte[2]);
        Wire2.write(data.MY_byte[1]);
        Wire2.write(data.MY_byte[0]);
        ack = Wire2.endTransmission(false);
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        Wire3.write(data.MY_byte[3]);
        Wire3.write(data.MY_byte[2]);
        Wire3.write(data.MY_byte[1]);
        Wire3.write(data.MY_byte[0]);
        ack = Wire3.endTransmission(false);
    }

    // ack = i2c_write_block_data(i2c_address, adc_command, (uint8_t) 4, data.LT_byte);

    return(ack);
}

// Reads an 8-bit adc_code from LTC2946
int8_t LTC2946::LTC2946_read(uint8_t adc_command, uint8_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    //  ack = i2c_read_byte_data(i2c_address, adc_command, adc_code);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        ack = Wire.endTransmission(false);

        Wire.requestFrom(I2C_ADDRESS, (uint8_t)1);

        *adc_code = Wire.read();
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        ack = Wire1.endTransmission(false);

        Wire1.requestFrom(I2C_ADDRESS, (uint8_t)1);

        *adc_code = Wire1.read();
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        ack = Wire2.endTransmission(false);

        Wire2.requestFrom(I2C_ADDRESS, (uint8_t)1);

        *adc_code = Wire2.read();
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        ack = Wire3.endTransmission(false);

        Wire3.requestFrom(I2C_ADDRESS, (uint8_t)1);

        *adc_code = Wire3.read();
    }

    return ack;
}

// Reads a 12-bit adc_code from LTC2946
int8_t LTC2946::LTC2946_read_12_bits(uint8_t adc_command, uint16_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    // Use union type defined in Linduino.h to combine two uint8_t's (8-bit unsigned integers) into one uint16_t (unsigned 16-bit integer)
    // Then, shift by 4 bits and return in *adc_code
    int8_t ack;

    union
    {
        uint8_t b[2];
        uint16_t w;
    } data;


    //  ack = i2c_read_word_data(i2c_address, adc_command, adc_code);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        ack = Wire.endTransmission(false);

        Wire.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire.read();
        data.b[0] = Wire.read();
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        ack = Wire1.endTransmission(false);

        Wire1.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire1.read();
        data.b[0] = Wire1.read();
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        ack = Wire2.endTransmission(false);

        Wire2.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire2.read();
        data.b[0] = Wire2.read();
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        ack = Wire3.endTransmission(false);

        Wire3.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire3.read();
        data.b[0] = Wire3.read();
    }

    //Serial.print(data.b[1],BIN); Serial.print(" | "); Serial.print(data.b[0],BIN); Serial.print(" | "); Serial.print(data.w,BIN); Serial.print(" | ");

    *adc_code = data.w;

    *adc_code >>= 4;
    return ack;
}

// Reads a 16-bit adc_code from LTC2946
int8_t LTC2946::LTC2946_read_16_bits(uint8_t adc_command, uint16_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    union
    {
        uint8_t b[2];
        uint16_t w;
    } data;

    // ack = i2c_read_word_data(i2c_address, adc_command, adc_code);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        ack = Wire.endTransmission(false);

        Wire.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire.read();
        data.b[0] = Wire.read();
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        ack = Wire1.endTransmission(false);

        Wire1.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire1.read();
        data.b[0] = Wire1.read();
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        ack = Wire2.endTransmission(false);

        Wire2.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire2.read();
        data.b[0] = Wire2.read();
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        ack = Wire3.endTransmission(false);

        Wire3.requestFrom(I2C_ADDRESS, (uint8_t)2);

        data.b[1] = Wire3.read();
        data.b[0] = Wire3.read();
    }

    *adc_code = data.w;

    return ack;
}

// Reads a 24-bit adc_code from LTC2946
int8_t LTC2946::LTC2946_read_24_bits(uint8_t adc_command, uint32_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    union
    {
        int32_t MY_int32;    //!< 32-bit signed integer to be converted to four bytes
        uint32_t MY_uint32;  //!< 32-bit unsigned integer to be converted to four bytes
        uint8_t MY_byte[4];  //!< 4 bytes (unsigned 8-bit integers) to be converted to a 32-bit signed or unsigned integer
    } data;

    // ack = i2c_read_block_data(i2c_address, adc_command, (uint8_t)3, data.LT_byte);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        ack = Wire.endTransmission(false);

        Wire.requestFrom(I2C_ADDRESS, (uint8_t)3);

        data.MY_byte[2] = Wire.read();
        data.MY_byte[1] = Wire.read();
        data.MY_byte[0] = Wire.read();
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        ack = Wire1.endTransmission(false);

        Wire1.requestFrom(I2C_ADDRESS, (uint8_t)3);

        data.MY_byte[2] = Wire1.read();
        data.MY_byte[1] = Wire1.read();
        data.MY_byte[0] = Wire1.read();
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        ack = Wire2.endTransmission(false);

        Wire2.requestFrom(I2C_ADDRESS, (uint8_t)3);

        data.MY_byte[2] = Wire2.read();
        data.MY_byte[1] = Wire2.read();
        data.MY_byte[0] = Wire2.read();
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        ack = Wire3.endTransmission(false);

        Wire3.requestFrom(I2C_ADDRESS, (uint8_t)3);

        data.MY_byte[2] = Wire3.read();
        data.MY_byte[1] = Wire3.read();
        data.MY_byte[0] = Wire3.read();
    }

    *adc_code = 0x0FFFFFF & data.MY_int32;
    return(ack);
}

// Reads a 32-bit adc_code from LTC2946
int8_t LTC2946::LTC2946_read_32_bits(uint8_t adc_command, uint32_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
    int8_t ack;

    union
    {
        int32_t MY_int32;    //!< 32-bit signed integer to be converted to four bytes
        uint32_t MY_uint32;  //!< 32-bit unsigned integer to be converted to four bytes
        uint8_t MY_byte[4];  //!< 4 bytes (unsigned 8-bit integers) to be converted to a 32-bit signed or unsigned integer
    } data;

    // ack = i2c_read_block_data(i2c_address, adc_command, (uint8_t) 4, data.LT_byte);

    if(I2C_WIRE == 0){
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(adc_command);

        ack = Wire.endTransmission(false);

        Wire.requestFrom(I2C_ADDRESS, (uint8_t)4);

        data.MY_byte[3] = Wire.read();
        data.MY_byte[2] = Wire.read();
        data.MY_byte[1] = Wire.read();
        data.MY_byte[0] = Wire.read();
    }else if(I2C_WIRE == 1){
        Wire1.beginTransmission(I2C_ADDRESS);
        Wire1.write(adc_command);

        ack = Wire1.endTransmission(false);

        Wire1.requestFrom(I2C_ADDRESS, (uint8_t)4);

        data.MY_byte[3] = Wire1.read();
        data.MY_byte[2] = Wire1.read();
        data.MY_byte[1] = Wire1.read();
        data.MY_byte[0] = Wire1.read();
    }else if(I2C_WIRE == 2){
        Wire2.beginTransmission(I2C_ADDRESS);
        Wire2.write(adc_command);

        ack = Wire2.endTransmission(false);

        Wire2.requestFrom(I2C_ADDRESS, (uint8_t)4);

        data.MY_byte[3] = Wire2.read();
        data.MY_byte[2] = Wire2.read();
        data.MY_byte[1] = Wire2.read();
        data.MY_byte[0] = Wire2.read();
    }else if(I2C_WIRE == 3){
        Wire3.beginTransmission(I2C_ADDRESS);
        Wire3.write(adc_command);

        ack = Wire3.endTransmission(false);

        Wire3.requestFrom(I2C_ADDRESS, (uint8_t)4);

        data.MY_byte[3] = Wire3.read();
        data.MY_byte[2] = Wire3.read();
        data.MY_byte[1] = Wire3.read();
        data.MY_byte[0] = Wire3.read();
    }



    *adc_code = 0xFFFFFFFF & data.MY_int32;
    return(ack);
}

// Calculate the LTC2946 VIN voltage
float LTC2946::LTC2946_VIN_code_to_voltage(uint16_t adc_code)
// Returns the VIN Voltage in Volts
//Legacy code
{
  float voltage;
  voltage = (float)adc_code*LTC2946_VIN_lsb;    //! 1) Calculate voltage from code and lsb
  return(voltage);
}

// Calculate the LTC2946 ADIN voltage
float LTC2946::LTC2946_ADIN_code_to_voltage(uint16_t adc_code)
// Returns the ADIN Voltage in Volts
//Legacy code
{
  float adc_voltage;
  adc_voltage = (float)adc_code*LTC2946_ADIN_lsb;   //! 1) Calculate voltage from code and ADIN lsb
  return(adc_voltage);
}

// Calculate the LTC2946 current with a sense resistor
float LTC2946::LTC2946_code_to_current(uint16_t adc_code)
// Returns the LTC2946 current in Amps
//Legacy code
{
  float voltage, current;
  voltage = (float)adc_code*LTC2946_DELTA_SENSE_lsb;    //! 1) Calculate voltage from ADC code and delta sense lsb
  current = voltage/resistor;                           //! 2) Calculate current, I = V/R
  return(current);
}

// Calculate the LTC2946 power
float LTC2946::LTC2946_code_to_power(int32_t adc_code)
// Returns The LTC2946 power in Watts
//Legacy code
{
  float power;
  power = (float)adc_code*LTC2946_Power_lsb/resistor;  //! 1) Calculate Power using Power lsb and resistor

  return(power);
}


// Calculate the LTC2946 energy
float LTC2946::LTC2946_code_to_energy(int32_t adc_code)
// Returns the LTC2946 energy in Joules
//Legacy code
{
  float energy_lsb, energy;
  energy_lsb=(float)(LTC2946_Power_lsb/resistor)*65536*LTC2946_TIME_lsb;   //! 1) Calculate Energy lsb from Power lsb and Time lsb
  energy = adc_code*energy_lsb;                               //! 2) Calculate Energy using Energy lsb and adc code
  return(energy);
}

// Calculate the LTC2946 Coulombs
float LTC2946::LTC2946_code_to_coulombs(int32_t adc_code)
// Returns the LTC2946 Coulombs
//Legacy code
{
  float coulomb_lsb, coulombs;
  coulomb_lsb=(float)(LTC2946_DELTA_SENSE_lsb/resistor)*16*LTC2946_TIME_lsb;   //! 1) Calculate Coulomb lsb Current lsb and Time lsb
  coulombs = adc_code*coulomb_lsb;                                             //! 2) Calculate Coulombs using Coulomb lsb and adc code
  return(coulombs);
}

//Calculate the LTC2946 Time in Seconds
float LTC2946::LTC2946_code_to_time(float time_code)
//Legacy code
{
  float seconds;
  seconds = LTC2946_TIME_lsb * time_code;
  return seconds;
}
