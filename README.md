# GPIO_Controller
 
 This project was done on CubeIDE for the STM32F407VGT6 with an ARM cortex-M4.
 It connects to a PC via USB and offers the following:

•	 Command line interface via USB (virtual com port).
•	 32 independent GPIO ports, each can be configured as input or output.
•	 32 independent logic channels, each can be set to ‘0’ or ‘1’ and configured to any of the 32 ports.
•	 An ADC calibrating to the internal reference voltage every second of the timer.
•	 2 ADC ports (PA1 and PA2) for measuring voltages, calculated from voltage dividers or linear approximations.
•	 An external interrupt with port debounce protection.
•	 Configurable ADC monitoring options for warning and error levels.
•	 Configurable sleep mode timer.
•	 Various status commands to display channels and ports configuration and restore default settings.
•	 Flash reading and writing options to save and recall settings.
•	 Up to 3 SPI master signals with latch lines.
