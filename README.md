# BaiMon

ESP8266 - based monitoring device for Vaillant TurboTec boilers.

Periodically monitors boiler internal state, water temperature and pressure.

Publishes measured results to 'narodmon.ru', TCP and MQTT protocols are supported.

Has its own web interface for viewing measured data and debugging EBus activity.

This version supports Vaillant TurboTec Pro. It can be adapted to other
boiler models using EBus interace. However, this requires knowledge of proprietary
commands to read parameter values.

Requires Arduino IDE with ESP8266 support to compile and upload to ESP8266-based controller (I run it on Wemos D1 Mini). 

Controller comminicates to the boiler using UART. Adapter is needed to adjust signal levels (EBus uses 9-20 volts).

EBus-adapter.pdf contains schematics of a simple EBus adapter.
It can be connected directly to ESP8266 outputs due to optical decoupling.

