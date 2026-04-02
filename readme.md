# Wallbox Arduino

Arduino Nano as simple wallbox controller


## Pinout

* D8: Relais control
* D10: PWM out for ControlPilot
* D11: LED stripe WS2812
* A1: ControlPilot feedback (56k to 5V, 100k to GND, 220k to CP.

## References:

* use case as simple PWM generator for pyPLC in EvseMode: https://github.com/uhi22/pyPLC/blob/master/hardware/plc_evse/plc_evse_schematic_v1.pdf
* use case as replacement controller in the innogy/eon ebox
