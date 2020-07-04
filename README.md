# Teensy-LTC2946
Adaptation of the LTC2946 I2C library for Teensy 3.6

Update of the excellent example coding for the LTC2946 power monitor by the OEM. Modified to work with Teensy 3.6 from the original code for Leonardo. 

-Relocated the majority of the functions required to access the LTC2946 into a class to isolate from main program. All I2C functions are now private. 
-Increased functionality on the Teensy 3.6, enabled use of all 4 I2C wires. 
-Removed the rather confusing I2C address selection of the original code. 
-Added conversions by experimental constants, where the constants are determined by comparison with a know meter according to the following equation:
             Measured value = LTC2946 Raw Value * Constant        Note: Each property (VIN, Current, Power) uses a unique constant.

Current functionality:
-Continuous reading has full functionality for VIN, Current, and Power measurment. 
-SnapShot reading has full functionality for VIN and Current. 

TODO:
-Finish incorporating SnapShot functionality into this library.
-Incorporate limit functionality. 
-Incorporate non-ground referenced measurement functionality.

Note:
Requires the upgraded wire library for the teensy, i2c_t3.h, to fully utilize. 
