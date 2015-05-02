# Tiva C Series Launchpad (TM4C123G) driver for nRF24L01 2.4GHz Wireless Transceivers. This library is currently a rough library port of maniacbug's RF24 library here:

https://github.com/maniacbug/RF24

It has been modified from the work that was posted here:

http://we.easyelectronics.ru/kisoft/ek-tm4c123gxl-tiva-c-nrf24l01-podklyuchaem-migriruem-biblioteku-ispolzuem.html

!!! This code probably contains more than a few bugs and was really quickly put together. It is a beta of a beta at best. Use at your own risk. !!!

The pinout in the ping pong demo is provided below. 

NRF24L01+       TM4C123GH6PM
1.      GND     GND
2.      3V3     3V3
3.      CE      PA6
4.      CSN     PA7
5.      SCK     PA2
6.      MOSI    PA5
7.      MISO    PA4
8.      IRQ     -
