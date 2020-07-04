#include "LTC2946.h"
//#include <Wire.h>
#include <i2c_t3.h>


LTC2946 LTC2946(0,0x6F); //Constructor. Format: LTC2946 <name>(I2C wire number,I2C address of LTC2946)

void setup() {
  Serial.begin(115200);             //! Initialize the serial port to the PC

  LTC2946.Setup(); //Initialize appropriate wire object
  //LTC2946.SetVINConst(0.02485474); //Set constant for VIN conversion to volts
  //LTC2946.SetAmperageConst(0.00119677419); //Set constant for Current conversion to amperes
  //LTC2946.SetPowerConst(0.00003171126055); //Set constant for Power conversion to watts

  LTC2946.SetContinuous(); //Set for continuous mode. Snapshot must be commented out, below. 
  //LTC2946.SetSnapShot(); //Set for SnapShot mode. Continuous must be commented out, above.
  LTC2946.EnableConversion(true); //Enable conversion from RAW to standard value
  LTC2946.EnableLegacy(false); //Enable legacy calculations using OEM LSB values. Disables VIN, Amperage, Power values set above.
  
}

void loop() {
  float VIN_voltage;
  float Power_watt;
  float Current_amps;

  VIN_voltage = LTC2946.ReadVIN();
  Current_amps = LTC2946.ReadCurrent();
  Power_watt = LTC2946.ReadPower();

  if(!LTC2946.ErrorCheck()){
    Serial.print("ERROR! | ");
  }

  Serial.print("VIN(v):"); Serial.print(VIN_voltage); Serial.print(" | Power: "); Serial.print(Power_watt); Serial.print(" | Current: "); Serial.println(Current_amps,4);
  delay(100);

}
