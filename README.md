# Squeak
GPS pet tracker. Details on the [hackaday.io](https://hackaday.io/project/188224-squeak-gps-pet-tracker) project page.


# Firmware
Squeak firmware is based on the [Microchip RN2483 firmware](https://github.com/MicrochipTech/RN2xx3_LORAWAN_FIRMWARE).
Project uses MPLABX v5.50 and XC8 v1.45 (free).
Apart from the application logic, the only change to the LoRaWAN stack is the addition of LORAWAN_CanSend() to lorawan.c, which tells if the stack can send a packet or not. The stack may be unable to tx because of duty cycle restrictions, and without LORAWAN_CanSend() the only way to tell is to actually transmit a packet. For Squeak this could be a problem if you just spent a lot of battery on getting a GPS fix and then cannot inform the user of this fresh location.
