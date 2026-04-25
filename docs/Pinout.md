# Pin Mapping

| MIC502 Pin | MIC502 Function | ATtiny25 Pin | ATtiny25 Function | Implementation |
|---:|---|---:|---|---|
| 1 | VT1 | 1 | PB5 / ADC0 | Thermistor analog input; requires RSTDISBL fuse |
| 2 | CF | N/A | Firmware timer | Replaced by Timer0 PWM time base |
| 3 | VSLP | N/A | Not used | Sleep function intentionally bypassed |
| 4 | GND | 4 | GND | Ground |
| 5 | VT2 | 5 | PB0 | Digital minimum-speed clamp input |
| 6 | /OTF | 6 | PB1 | Active-low open-drain-style output |
| 7 | OUT | 7 | PB2 | PWM output |
| 8 | VDD | 8 | VCC | 5 V supply |
