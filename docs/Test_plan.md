
# Test Plan

## PWM Output

- [ ] Verify PWM output on pin 7
- [ ] Measure PWM frequency
- [ ] Verify duty-cycle response from 0% to 100%

## VT1 Input

- [ ] Measure VT1 voltage at room temperature
- [ ] Verify ADC response as thermistor temperature increases
- [ ] Verify 30% VDD maps near 0% duty
- [ ] Verify 70% VDD maps near 100% duty

## VT2 Clamp

- [ ] Verify clamp defaults ON after power-up
- [ ] Verify clamp forces minimum 25% duty
- [ ] Verify push button toggles clamp OFF
- [ ] Verify push button toggles clamp ON again

## Startup Interval

- [ ] Verify output is forced HIGH after power-up
- [ ] Verify startup lasts 64 PWM cycles

## /OTF

- [ ] Verify /OTF LED turns on above threshold
- [ ] Verify /OTF output behaves active-low

## Shutdown / Reset

- [ ] Disable clamp
- [ ] Pull VT1 below VIL and verify shutdown
- [ ] Raise VT1 above VIH and verify restart
- [ ] Verify 64-cycle startup after restart
