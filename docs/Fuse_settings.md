# Fuse Settings

## Target

ATtiny25 running at internal 8 MHz with physical pin 1 used as ADC0.

## Fuse Values

| Fuse | Value | Purpose |
|---|---:|---|
| Low Fuse | 0xE2 | Internal 8 MHz oscillator, CKDIV8 disabled |
| High Fuse | 0x5F | RSTDISBL programmed, RESET disabled |
| Extended Fuse | 0xFF | Default extended fuse setting |

## Important Warning

Programming the RSTDISBL fuse disables the RESET function on physical pin 1.

After this fuse is programmed, normal ISP programming will no longer work. Use HVSP / High-Voltage Serial Programming for programming or recovery.
