/*
 * File: attiny25_mic502_replacement_final.c
 *
 * Target: ATtiny25
 * IDE: MPLAB X IDE
 * Programmer: HVSP / High-Voltage Serial Programming
 *
 * Pinout:
 * Pin 1 / PB5 / ADC0 : VT1 analog input, RESET disabled by RSTDISBL fuse
 * Pin 5 / PB0        : VT2 clamp momentary button input
 * Pin 6 / PB1        : /OTF active-low open-drain-style output
 * Pin 7 / PB2        : PWM OUT
 * Pin 8              : VCC
 * Pin 4              : GND
 *
 * Logic:
 * - Clamp starts ON after every power-up.
 * - Startup forces PWM OUT to 100% for 64 PWM cycles.
 * - Clamp ON: shutdown/reset ignored; PWM = max(VT1 duty, 25%).
 * - Clamp OFF: PWM follows VT1 fully; VT1 < VIL enters shutdown.
 * - Shutdown exits when VT1 > VIH, then startup runs again.
 * - /OTF asserts LOW when VT1 exceeds OTF threshold.
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/fuse.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------- FUSE CONFIGURATION ---------- */
/*
 * LOW FUSE  = 0xE2:
 * System level: ATtiny25 runs from internal 8 MHz oscillator with CKDIV8 disabled.
 * Internal level: CKSEL selects calibrated internal RC; CKDIV8 unprogrammed.
 *
 * HIGH FUSE = 0x5F:
 * System level: physical pin 1 becomes PB5/ADC0 instead of RESET.
 * Internal level: RSTDISBL is programmed.
 *
 * WARNING:
 * After RSTDISBL is programmed, normal ISP cannot enter programming mode.
 * Use HVSP to recover or reprogram fuses.
 */
FUSES = {
    .low      = 0xE2,
    .high     = 0x5F,
    .extended = 0xFF,
};

/* ---------- ADC THRESHOLDS ---------- */

#define ADC_FULL_SCALE 1023u

#define ADC_VPWM_MIN   ((uint16_t)(0.30 * ADC_FULL_SCALE + 0.5))
#define ADC_VPWM_MAX   ((uint16_t)(0.70 * ADC_FULL_SCALE + 0.5))
#define ADC_VPWM_SPAN  ((uint16_t)(ADC_VPWM_MAX - ADC_VPWM_MIN))

#define ADC_OTF        ((uint16_t)(0.77 * ADC_FULL_SCALE + 0.5))
/* For final MIC502-like threshold, change 0.50 to 0.75 or 0.77. */

#define ADC_VT1_VIL    ((uint16_t)((0.7 / 5.0) * ADC_FULL_SCALE + 0.5))
#define ADC_VT1_VIH    ((uint16_t)((1.1 / 5.0) * ADC_FULL_SCALE + 0.5))

/* ---------- PIN DEFINITIONS ---------- */

#define CH_VT1          0u
#define VT2_BUTTON_PIN  PB0
#define OTF_PIN         PB1
#define OUT_PIN         PB2

/* ---------- PWM CONFIGURATION ---------- */

#define PWM_TOP_COUNT       255u
#define MIN_DUTY_CLAMP      64u
#define STARTUP_CYCLES      64u

/*
 * Timer0 CTC interrupt frequency:
 * F_CPU / 8 / (OCR0A + 1) = 8 MHz / 8 / 39 = 25.641 kHz
 *
 * Software PWM frequency:
 * 25.641 kHz / 256 = about 100.16 Hz
 */
#define TIMER0_CTC_TOP      38u

/* ---------- GLOBAL STATE ---------- */

volatile uint8_t g_pwm_phase = 0;
/* System level: current PWM step inside one 100 Hz PWM period.
 * Internal level: incremented in Timer0 compare ISR from 0 to 255.
 */

volatile uint8_t g_next_duty = 0;
/* System level: duty command calculated by main control loop.
 * Internal level: written by main code, latched by ISR at PWM frame boundary.
 */

volatile uint8_t g_active_duty = 0;
/* System level: duty currently being output.
 * Internal level: read only inside ISR for PB2 switching.
 */

volatile uint8_t g_startup_cycles = STARTUP_CYCLES;
/* System level: remaining 100% startup PWM cycles.
 * Internal level: decremented once per PWM frame by ISR.
 */

volatile uint8_t g_control_step_req = 0;
/* System level: asks main loop to run control once per PWM cycle.
 * Internal level: set by ISR at PWM frame boundary.
 */

static uint16_t g_avg_vt1 = 0;
/* System level: filtered VT1 voltage.
 * Internal level: IIR filtered ADC0 value.
 */

static bool g_clamp_enabled = true;
/* System level: minimum fan speed clamp starts ON after power-up.
 * Internal level: firmware latch toggled by PB0 button.
 */

static bool g_shutdown_state = false;
/* System level: remembers shutdown mode when clamp is OFF.
 * Internal level: firmware state variable.
 */

static bool g_otf_active = false;
/* System level: remembers /OTF state.
 * Internal level: used for threshold hysteresis extension if needed.
 */

static bool g_last_button_state = false;
/* System level: previous button state for edge detection.
 * Internal level: stored PB0 state from previous control step.
 */

static uint8_t g_button_debounce = 0;
/* System level: prevents switch bounce from multiple toggles.
 * Internal level: countdown in PWM-control cycles.
 */

/* ---------- I/O HELPERS ---------- */

static void otf_release(void) {
    PORTB &= (uint8_t)~(1u << OTF_PIN);
    /* System level: disables pull-up/high drive on /OTF.
     * Internal level: clears PB1 PORT latch.
     */

    DDRB &= (uint8_t)~(1u << OTF_PIN);
    /* System level: releases /OTF to high impedance.
     * Internal level: configures PB1 as input, open-drain inactive.
     */
}

static void otf_assert_low(void) {
    PORTB &= (uint8_t)~(1u << OTF_PIN);
    /* System level: prepares /OTF to pull low.
     * Internal level: clears PB1 PORT latch.
     */

    DDRB |= (uint8_t)(1u << OTF_PIN);
    /* System level: asserts /OTF active-low; red LED turns ON if wired to VCC.
     * Internal level: configures PB1 as output LOW.
     */
}

static void out_enable(void) {
    DDRB |= (uint8_t)(1u << OUT_PIN);
    /* System level: enables PWM OUT driver.
     * Internal level: configures PB2 as output.
     */
}

static void out_force_low(void) {
    g_next_duty = 0;
    /* System level: requests 0% PWM.
     * Internal level: next PWM frame latches duty = 0.
     */

    PORTB &= (uint8_t)~(1u << OUT_PIN);
    /* System level: immediately turns PWM OUT off.
     * Internal level: clears PB2 PORT latch.
     */
}

static bool vt2_button_high(void) {
    return (PINB & (1u << VT2_BUTTON_PIN)) != 0;
    /* System level: returns true when button drives pin 5 HIGH.
     * Internal level: reads PB0 bit from PINB.
     */
}

/* ---------- ADC ---------- */

static void adc_init(void) {
    ADMUX = CH_VT1;
    /* System level: selects VT1 input on physical pin 1.
     * Internal level: selects ADC0 channel.
     */

    ADCSRA = (1u << ADEN) | (1u << ADPS2) | (1u << ADPS1);
    /* System level: enables ADC.
     * Internal level: ADC enabled, prescaler = 64; ADC clock = 125 kHz at 8 MHz CPU.
     */

#ifdef ADC0D
    DIDR0 |= (1u << ADC0D);
    /* System level: reduces digital switching noise on VT1 pin.
     * Internal level: disables digital input buffer on ADC0/PB5.
     */
#endif
}

static uint16_t adc_read_channel(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0u) | (ch & 0x0Fu);
    /* System level: selects requested analog input.
     * Internal level: writes ADC multiplexer bits.
     */

    ADCSRA |= (1u << ADSC);
    /* System level: starts dummy ADC conversion after channel selection.
     * Internal level: sets ADC start conversion bit.
     */

    while (ADCSRA & (1u << ADSC)) {
    }
    /* System level: waits for dummy conversion to finish.
     * Internal level: hardware clears ADSC when done.
     */

    (void)ADC;
    /* System level: discards first sample after MUX change.
     * Internal level: reads ADC register to clear result path.
     */

    ADCSRA |= (1u << ADSC);
    /* System level: starts real ADC conversion.
     * Internal level: sets ADSC again.
     */

    while (ADCSRA & (1u << ADSC)) {
    }
    /* System level: waits for real conversion result.
     * Internal level: polling ADSC bit.
     */

    return ADC;
    /* System level: returns ADC result from 0 to 1023.
     * Internal level: reads ADCL/ADCH through ADC macro.
     */
}

static void adc_update_filter(uint8_t ch, uint16_t *avg_store) {
    uint16_t raw = adc_read_channel(ch);
    /* System level: obtains fresh VT1 sample.
     * Internal level: calls ADC conversion routine.
     */

    if (*avg_store == 0u) {
        *avg_store = raw;
        /* System level: initializes filter with first reading.
         * Internal level: stores raw ADC count.
         */
    } else {
        *avg_store = (uint16_t)(((*avg_store) * 3u + raw) >> 2);
        /* System level: smooths VT1 noise with 75% old + 25% new filter.
         * Internal level: integer IIR filter using shift divide by 4.
         */
    }
}

/* ---------- MATH ---------- */

static uint8_t map_vt1_to_duty(uint16_t vt1_adc) {
    if (vt1_adc <= ADC_VPWM_MIN) {
        return 0u;
        /* System level: VT1 below 30% VDD requests 0% duty.
         * Internal level: lower saturation.
         */
    }

    if (vt1_adc >= ADC_VPWM_MAX) {
        return 255u;
        /* System level: VT1 above 70% VDD requests 100% duty.
         * Internal level: upper saturation.
         */
    }

    return (uint8_t)(((uint32_t)(vt1_adc - ADC_VPWM_MIN) * 255u) / ADC_VPWM_SPAN);
    /* System level: maps VT1 linearly from 30?70% VDD to 0?100% PWM.
     * Internal level: 32-bit multiply avoids overflow, returns 8-bit duty.
     */
}

/* ---------- BUTTON / CLAMP ---------- */

static void update_clamp_button(void) {
    bool current = vt2_button_high();
    /* System level: samples physical pin 5 button/comparator flag.
     * Internal level: reads PB0 input latch.
     */

    if (g_button_debounce > 0u) {
        g_button_debounce--;
        /* System level: waits out mechanical switch bounce.
         * Internal level: decrements debounce counter once per PWM cycle.
         */
    }

    if (current && !g_last_button_state && (g_button_debounce == 0u)) {
        g_clamp_enabled = !g_clamp_enabled;
        /* System level: button press toggles minimum clamp ON/OFF.
         * Internal level: flips boolean state.
         */

        g_button_debounce = 5u;
        /* System level: about 50 ms debounce at 100 Hz control rate.
         * Internal level: stores lockout count.
         */

        if (g_clamp_enabled && g_shutdown_state) {
            g_shutdown_state = false;
            g_startup_cycles = STARTUP_CYCLES;
            out_enable();
            /* System level: if clamp is turned ON while shut down, restart fan.
             * Internal level: clears shutdown flag and reloads startup counter.
             */
        }
    }

    g_last_button_state = current;
    /* System level: saves button state for next edge detection.
     * Internal level: updates boolean memory.
     */
}

/* ---------- CONTROL LOGIC ---------- */

static void update_otf(uint16_t vt1_adc) {
    if (vt1_adc >= ADC_OTF) {
        g_otf_active = true;
        /* System level: VT1 exceeds OTF threshold.
         * Internal level: sets fault state variable.
         */
    } else {
        g_otf_active = false;
        /* System level: VT1 below OTF threshold.
         * Internal level: clears fault state variable.
         */
    }

    if (g_otf_active) {
        otf_assert_low();
        /* System level: /OTF LED/fault turns ON.
         * Internal level: PB1 sinks current.
         */
    } else {
        otf_release();
        /* System level: /OTF LED/fault turns OFF.
         * Internal level: PB1 high-Z.
         */
    }
}

static void mic502_control_step(void) {
    adc_update_filter(CH_VT1, &g_avg_vt1);
    /* System level: reads and filters VT1 thermistor voltage.
     * Internal level: ADC0 conversion plus IIR filter.
     */

    uint16_t vt1 = g_avg_vt1;
    /* System level: uses filtered VT1 for all decisions.
     * Internal level: copies filter state into local variable.
     */

    update_clamp_button();
    /* System level: allows pin 5 button to toggle clamp.
     * Internal level: reads PB0 and updates latch.
     */

    if (!g_clamp_enabled) {
        /* System level: shutdown/reset is active only when clamp is OFF.
         * Internal level: branches based on clamp boolean.
         */

        if (!g_shutdown_state && (vt1 <= ADC_VT1_VIL)) {
            g_shutdown_state = true;
            g_startup_cycles = 0u;
            out_force_low();
            otf_release();
            return;
            /* System level: VT1 low shuts fan controller down.
             * Internal level: sets shutdown flag, cancels startup, clears PB2 and releases PB1.
             */
        }

        if (g_shutdown_state) {
            if (vt1 >= ADC_VT1_VIH) {
                g_shutdown_state = false;
                g_startup_cycles = STARTUP_CYCLES;
                out_enable();
                otf_release();
                /* System level: VT1 recovery exits shutdown and forces startup.
                 * Internal level: clears shutdown state and reloads startup counter.
                 */
            } else {
                out_force_low();
                otf_release();
                return;
                /* System level: remains shut down while VT1 is still low.
                 * Internal level: keeps PB2 off and PB1 high-Z.
                 */
            }
        }
    } else {
        if (g_shutdown_state) {
            g_shutdown_state = false;
            g_startup_cycles = STARTUP_CYCLES;
            out_enable();
            /* System level: clamp ON overrides shutdown mode and restarts fan.
             * Internal level: clears shutdown flag and reloads startup counter.
             */
        }
    }

    update_otf(vt1);
    /* System level: updates active-low overtemperature output.
     * Internal level: drives or releases PB1.
     */

    uint8_t duty = map_vt1_to_duty(vt1);
    /* System level: converts VT1 voltage to PWM duty.
     * Internal level: calculates 8-bit duty value.
     */

    if (g_clamp_enabled && (duty < MIN_DUTY_CLAMP)) {
        duty = MIN_DUTY_CLAMP;
        /* System level: clamp ON enforces minimum 25% duty.
         * Internal level: replaces duty with 64/255.
         */
    }

    out_enable();
    /* System level: ensures PWM output pin is actively driven.
     * Internal level: sets PB2 as output.
     */

    g_next_duty = duty;
    /* System level: commands next PWM duty.
     * Internal level: writes double buffer used by ISR.
     */
}

/* ---------- TIMER0 ISR PWM ENGINE ---------- */

ISR(TIMER0_COMPA_vect) {
    g_pwm_phase++;
    /* System level: advances software PWM phase.
     * Internal level: increments 8-bit phase counter.
     */

    if (g_pwm_phase == 0u) {
        uint8_t duty_snapshot = g_next_duty;
        /* System level: latches new duty once per PWM period.
         * Internal level: copies volatile double buffer.
         */

        if (g_startup_cycles > 0u) {
            duty_snapshot = 255u;
            g_startup_cycles--;
            /* System level: startup forces 100% duty for 64 cycles.
             * Internal level: overrides duty and decrements startup counter once per PWM frame.
             */
        }

        g_active_duty = duty_snapshot;
        g_control_step_req = 1u;
        /* System level: asks main loop to update control once per PWM period.
         * Internal level: stores active duty and sets volatile flag.
         */
    }

    if (g_active_duty == 0u) {
        PORTB &= (uint8_t)~(1u << OUT_PIN);
        /* System level: 0% duty keeps output LOW.
         * Internal level: clears PB2.
         */
    } else if (g_active_duty == 255u) {
        PORTB |= (uint8_t)(1u << OUT_PIN);
        /* System level: 100% duty keeps output HIGH.
         * Internal level: sets PB2.
         */
    } else if (g_pwm_phase < g_active_duty) {
        PORTB |= (uint8_t)(1u << OUT_PIN);
        /* System level: PWM ON portion.
         * Internal level: sets PB2 while phase is below duty.
         */
    } else {
        PORTB &= (uint8_t)~(1u << OUT_PIN);
        /* System level: PWM OFF portion.
         * Internal level: clears PB2 after phase reaches duty.
         */
    }
}

/* ---------- HARDWARE INIT ---------- */

static void hw_init(void) {
    cli();
    /* System level: prevents interrupts during setup.
     * Internal level: clears global interrupt enable bit.
     */

    out_enable();
    out_force_low();
    /* System level: initializes PWM output OFF.
     * Internal level: PB2 output, duty buffer zero.
     */

    DDRB &= (uint8_t)~(1u << VT2_BUTTON_PIN);
    PORTB &= (uint8_t)~(1u << VT2_BUTTON_PIN);
    /* System level: pin 5 is input; external 10 k? pulldown and button-to-VCC used.
     * Internal level: PB0 input, internal pull-up disabled.
     */

    otf_release();
    /* System level: /OTF starts inactive.
     * Internal level: PB1 high-Z.
     */

    adc_init();
    /* System level: initializes VT1 ADC0 on physical pin 1.
     * Internal level: ADC enabled and ADC0 digital buffer disabled.
     */

    TCCR0A = (1u << WGM01);
    /* System level: Timer0 creates fixed interrupt tick.
     * Internal level: CTC mode using OCR0A as TOP.
     */

    TCCR0B = (1u << CS01);
    /* System level: Timer0 clock = F_CPU / 8.
     * Internal level: prescaler bits select divide-by-8.
     */

    OCR0A = TIMER0_CTC_TOP;
    /* System level: sets interrupt rate for about 100 Hz PWM with 256 steps.
     * Internal level: compare match occurs every 39 timer counts.
     */

    TIMSK |= (1u << OCIE0A);
    /* System level: enables Timer0 compare interrupt.
     * Internal level: sets OCIE0A interrupt enable bit.
     */

    set_sleep_mode(SLEEP_MODE_IDLE);
    /* System level: CPU can sleep between PWM/control events.
     * Internal level: idle mode leaves Timer0 and ADC clocks available.
     */

    sei();
    /* System level: starts interrupt-driven PWM engine.
     * Internal level: sets global interrupt enable bit.
     */
}

/* ---------- MAIN ---------- */

int main(void) {
    hw_init();
    /* System level: initializes I/O, ADC, Timer0, PWM, and /OTF.
     * Internal level: configures AVR registers.
     */

    g_clamp_enabled = true;
    g_shutdown_state = false;
    g_startup_cycles = STARTUP_CYCLES;
    g_next_duty = MIN_DUTY_CLAMP;
    /* System level: every power-up starts with clamp ON and startup active.
     * Internal level: initializes firmware state variables.
     */

    while (1) {
        if (g_control_step_req) {
            g_control_step_req = 0u;
            mic502_control_step();
            /* System level: updates fan control once per PWM period.
             * Internal level: clears ISR flag and runs ADC/control logic.
             */
        }

        sleep_mode();
        /* System level: saves power between interrupts.
         * Internal level: CPU sleeps in idle mode until Timer0 wakes it.
         */
    }
}