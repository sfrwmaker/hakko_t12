Soldering IRON controller fot hakko t12 tips based on atmega328p-pu muicrocontroller (Arduino UNO/NANO)

Updated 09/05/2019. Preset temperature jumps when finished rotating the encoder issue fixed

Updated 04/18/2019. Two display configuration support added. You should select appropriate display in the config.h file

Updated 01/14/2018. Fixed bug with the IRON overheat. New procedure in the tip configutration implemented. 

Updated 11/23/2017. Fixed bug with cold temperature detection. New criteria for ready state implemented in the working mode and in the tip configuration mode.

Updated 10/21/2017. New PID parameters used. 

Updated 10/14/2017. Fixed unstable work with 'thin' soldering tips such as T12-BC1, T12-JL02, T12-IL, T12-D12 by Increasing the frequency of temperature sampling to 50 Hz. New PID coefficients implemented.

Updated 7/1/2017. New software features are available: new per-tip calibration and high-frequency temperature check by Timer1, now no addititional software library required. 
