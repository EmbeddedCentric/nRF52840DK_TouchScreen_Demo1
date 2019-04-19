A simple demo program that utilizes the nRF52840 DK, and gen4-uLCD-24PT touch screen by 4D Systems.

The demo supports the following: 
1. Control LED4 on the nRF52840 DK through both the touch screen and BLE.
2. Get the status of Button 4 on the nRF52840 DK through both the touch screen and BLE.
3. Read the nRF52840 on-chip tempreture sensor through both the touch screen and BLE.
The demo comes with a custom made Android App to interact with the board. nRF Connect could also be used.
The demo creates a NUS BLE service. NUS stands for  Nordic UART service.
NUS sets up one "RX" characteristic with "write" properties, and one "TX" characteristic with "notify" properties datachannel, to fit basic communication needs.
The LED4 is controlled through the RX characteristics writing  (0x46 == ON) and (0x4F == OFF).
When notification is enabled on the TX characteristics, pressing Button4 will send the string "Button 4 Pressed". In addition the tempreture of the chip is sent over the TX characteristics every TEMP_RADING_INTERVAL time

Interface to the touch screen is serial following the Genie Standard Protocol.
LCD RX -> p0.27
LCD TX -> p0.26

Important notes:
*Segger Embedded Studio V4.12 is used to build this demo. nRF5 SDK 15.2.0 and above is needed.
*Double check that the nRF5SDK global macro is pointing to the right location where the nRF5 SDK is downloaded on your local disk. This can be done by -
 checking  Tools Menu -> Options -> Building -> Global Macros (nRF5SDK) and compare it to the root directory where the nRF5 SDK is downloaded
*This demo is part of Nordic nRF5x BLE In-Depth Training Course -Intermediate Level ( Optional extracurricular activity after lesson 12)