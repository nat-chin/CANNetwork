## CAN Network link to IoT system.

### CAN Network consist of MPU6050(IMU) , GPS module , voltage and Current Sensor
1.ESP32 (Head Unit) : 
2.Arduino UNO R3 or Nano : IMU data
3.Arduino UNO R3 or Nano : RPM , Nominal Voltage of Baterry , Current to Load (Motor)
(3) may update to PI PICO 

ID priority (Non-Header Node)
    From GPS Node
    Lat : 0xF1
    Lng : 0xF2 

    From Motor Controller Node
    RPM : 0xF3
    Accelx  : 0xF4
    Accely  : 0xF5
    AccelZ  : 0xF6
    GyroX   : 0xF7
    GyroY   : 0xF8
    GyroZ   : 0xF9

    From This head unit
    Monitor : Nominal Voltage of Battery the voltage (If possible Calculate for SOC and SOH )
              Current (Discharge) of Battery to Motor

    So the order of received message in 500 Kbps (with 100ms timer delay) (Sampling Rate) is 
    Lat Lng RPM Accelx , Accely , Accelz , Gyro x , Gyro y , Gyro z

### PUT The block Diagram and Flow of Phase 1 project
### Block Diagram of Whole Project

### Journey Through each phase

### Schematic or Image of Pinout