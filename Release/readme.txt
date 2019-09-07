1、LM502 LoRaWAN Module v1.0: Firmware to use LM502 as a LoRaWAN AT Command module. 
	* Doesn't support any sensor
	* Need external MCU to use AT command to start LoRaWAN
2、LM502 LoRaWAN Sensor Node v1.0: Firmware to use LM502 as a standalone module. 
	* Support various sensors
	* no need external MCU

3、Pingpong test: Peer to Peer test. No LoRaWAN, can be used for all frequency

Important Notice: 
a) After upgrade firmware. Reset the LM502 and will be in low power mode. 
b) Recommend enter AT Command twice for the first AT command: the first is to wake up the module. After all AT Command is set, need to run AT+CLPM=1 to enter low power mode. otherwise the power consumption will be 2.8mA. In Low power mode, power consumption is about 4.3uA
c) When connect I2C device, because the Demo Board doesn't have pull up resistor (10K). The power will be dozens uA. after add two 10k resistors. It will back to 4.3uA
d) P6.4 is the bootloader flag pin. It will enter bootloader if P6.4 is low. If there is trouble to upload firmware. Set this pin to low will be ok. Normally this pin need to add a 10K pull up to VCC.
e) While design hardware for LM502 as a AT module. Recommend to add 10K pull up for P3.0/P3.1/P3.2/P3.3 . P6.4 connect to MCU GPIO, to ensure the upload firmware can be controlled by MCU 
