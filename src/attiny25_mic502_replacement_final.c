/*
 * File: attiny25_mic502_replacement_final.c
 *
 * Target MCU:
 *   ATtiny25, DIP-8 package.
 *
 * Development Environment:
 *   MPLAB X IDE with AVR-GCC / XC8 AVR-compatible toolchain.
 *
 * Programming Method:
 *   HVSP / High-Voltage Serial Programming.
 *
 * Hardware Pin Mapping:
 *   Physical Pin 1 / PB5 / ADC0:
 *     Circuit/System level:
 *       VT1 thermistor-conditioned analog voltage input.
 *       This is the main temperature-control input.
 *     ATtiny25/Internal level:
 *       ADC0 input channel on PB5.
 *       RESET function must be disabled through the RSTDISBL fuse.
 *
 *   Physical Pin 5 / PB0:
 *     Circuit/System level:
 *       VT2 digital clamp button input.
 *       A momentary push button toggles minimum fan-speed clamp ON/OFF.
 *     ATtiny25/Internal level:
 *       PB0 is configured as a digital input and read through PINB.
 *
 *   Physical Pin 6 / PB1:
 *     Circuit/System level:
 *       /OTF active-low overtemperature fault output.
 *       Intended to behave like an open-collector/open-drain fault output.
 *     ATtiny25/Internal level:
 *       PB1 is switched between high-impedance input mode and output-low mode.
 *
 *   Physical Pin 7 / PB2:
 *     Circuit/System level:
 *       PWM fan-drive output.
 *       In testing, this can drive an LED through a resistor.
 *       In the final circuit, this should drive a MOSFET/transistor gate/base stage.
 *     ATtiny25/Internal level:
 *       PB2 is driven by an interrupt-based software PWM engine.
 *
 *   Physical Pin 8:
 *     Circuit/System level:
 *       VCC supply input.
 *
 *   Physical Pin 4:
 *     Circuit/System level:
 *       Ground reference.
 *
 * Main Firmware Behavior:
 *   - Minimum fan-speed clamp starts ON after every power-up.
 *   - Startup interval forces PWM OUT to 100% duty for 64 PWM cycles.
 *   - When clamp is ON:
 *       shutdown/reset is ignored;
 *       PWM duty = max(VT1-derived duty, 25%).
 *   - When clamp is OFF:
 *       PWM follows VT1 completely;
 *       VT1 below VIL enters shutdown;
 *       VT1 above VIH exits shutdown and triggers startup again.
 *   - /OTF pulls LOW when VT1 exceeds the overtemperature threshold.
 */

#define F_CPU 8000000UL
/* Circuit/System level:
 *   Declares that the firmware expects an 8 MHz CPU clock.
 *   This matches the intended internal oscillator fuse configuration.
 * ATtiny25/Internal level:
 *   Used by compiler libraries and timing-related code.
 *   Even though this firmware mainly uses Timer0 interrupts, this value should still match the real clock.
 */

#include <avr/io.h>
/* Circuit/System level:
 *   Allows firmware to control ATtiny25 hardware pins and peripherals.
 * ATtiny25/Internal level:
 *   Provides definitions for DDRB, PORTB, PINB, ADMUX, ADCSRA, TCCR0A, TCCR0B, OCR0A, TIMSK, etc.
 */

#include <avr/interrupt.h>
/* Circuit/System level:
 *   Allows firmware to use interrupt-driven PWM timing.
 * ATtiny25/Internal level:
 *   Provides ISR definitions, cli(), and sei().
 */

#include <avr/sleep.h>
/* Circuit/System level:
 *   Allows the CPU to sleep between PWM/control events to reduce unnecessary activity.
 * ATtiny25/Internal level:
 *   Provides set_sleep_mode() and sleep_mode().
 */

#include <avr/fuse.h>
/* Circuit/System level:
 *   Allows fuse values to be embedded in the compiled output.
 * ATtiny25/Internal level:
 *   Provides the FUSES structure used by the AVR toolchain.
 */

#include <stdint.h>
/* Circuit/System level:
 *   Provides explicit integer sizes for embedded firmware reliability.
 * ATtiny25/Internal level:
 *   Defines uint8_t, uint16_t, and uint32_t.
 */

#include <stdbool.h>
/* Circuit/System level:
 *   Allows true/false logic for firmware states such as clamp_enabled and shutdown_state.
 * ATtiny25/Internal level:
 *   Defines bool, true, and false.
 */

/* ---------- FUSE CONFIGURATION ---------- */

/*
 * LOW FUSE = 0xE2:
 * Circuit/System level:
 *   Selects internal 8 MHz clock operation with clock divide-by-8 disabled.
 *   The firmware timing and Timer0 PWM calculations assume this 8 MHz clock.
 * ATtiny25/Internal level:
 *   Configures internal RC oscillator selection and leaves CKDIV8 unprogrammed.
 *
 * HIGH FUSE = 0x5F:
 * Circuit/System level:
 *   Disables RESET on physical pin 1 so that pin 1 can be used as VT1 ADC input.
 * ATtiny25/Internal level:
 *   Programs RSTDISBL so PB5/ADC0 becomes available instead of acting as RESET.
 *
 * WARNING:
 * Circuit/System level:
 *   Once RESET is disabled, the board cannot be programmed through normal ISP.
 * ATtiny25/Internal level:
 *   Serial programming entry through RESET is no longer available.
 *   HVSP is required for programming, fuse recovery, or firmware update.
 */
FUSES = {
    .low      = 0xE2,
    .high     = 0x5F,
    .extended = 0xFF,
};

/* ---------- ADC THRESHOLDS ---------- */

#define ADC_FULL_SCALE 1023u
/* Circuit/System level:
 *   Represents the maximum count of a 10-bit ADC conversion.
 *   0 means 0 V and 1023 means approximately VCC when VCC is used as ADC reference.
 * ATtiny25/Internal level:
 *   Used as the numeric full-scale value for ADC threshold calculations.
 */

#define ADC_VPWM_MIN   ((uint16_t)(0.30 * ADC_FULL_SCALE + 0.5))
/* Circuit/System level:
 *   VT1 at 30% of VDD corresponds to 0% PWM duty in the MIC502-style mapping.
 * ATtiny25/Internal level:
 *   Converts 0.30 × 1023 into an integer ADC threshold.
 */

#define ADC_VPWM_MAX   ((uint16_t)(0.70 * ADC_FULL_SCALE + 0.5))
/* Circuit/System level:
 *   VT1 at 70% of VDD corresponds to 100% PWM duty in the MIC502-style mapping.
 * ATtiny25/Internal level:
 *   Converts 0.70 × 1023 into an integer ADC threshold.
 */

#define ADC_VPWM_SPAN  ((uint16_t)(ADC_VPWM_MAX - ADC_VPWM_MIN))
/* Circuit/System level:
 *   Defines the active VT1 control range between 30% and 70% of VDD.
 * ATtiny25/Internal level:
 *   Used as the denominator for linear ADC-to-duty mapping.
 */

#define ADC_OTF        ((uint16_t)(0.77 * ADC_FULL_SCALE + 0.5))
/* Circuit/System level:
 *   Defines the overtemperature threshold at about 77% of VDD.
 *   When VT1 exceeds this level, /OTF is asserted LOW.
 * ATtiny25/Internal level:
 *   Converts 0.77 × 1023 into a 10-bit ADC comparison threshold.
 */

#define ADC_VT1_VIL    ((uint16_t)((0.7 / 5.0) * ADC_FULL_SCALE + 0.5))
/* Circuit/System level:
 *   Defines the MIC502-like VT1 low-input shutdown threshold.
 *   When clamp is OFF and VT1 falls below this threshold, the controller shuts down.
 *   This calculation assumes a nominal 5.0 V ADC reference.
 * ATtiny25/Internal level:
 *   Converts 0.7 V into an ADC count using 5.0 V as the reference assumption.
 */

#define ADC_VT1_VIH    ((uint16_t)((1.1 / 5.0) * ADC_FULL_SCALE + 0.5))
/* Circuit/System level:
 *   Defines the MIC502-like VT1 high-input restart threshold.
 *   When clamp is OFF and VT1 rises above this threshold, the controller restarts.
 *   This calculation assumes a nominal 5.0 V ADC reference.
 * ATtiny25/Internal level:
 *   Converts 1.1 V into an ADC count using 5.0 V as the reference assumption.
 */

/* ---------- PIN DEFINITIONS ---------- */

#define CH_VT1          0u
/* Circuit/System level:
 *   Assigns VT1 to ADC channel 0, which is physical pin 1 after RESET is disabled.
 * ATtiny25/Internal level:
 *   ADC multiplexer channel number for ADC0.
 */

#define VT2_BUTTON_PIN  PB0
/* Circuit/System level:
 *   Assigns the VT2 clamp button/flag to physical pin 5.
 * ATtiny25/Internal level:
 *   PB0 is used as a digital input bit in PINB, PORTB, and DDRB.
 */

#define OTF_PIN         PB1
/* Circuit/System level:
 *   Assigns /OTF overtemperature fault output to physical pin 6.
 * ATtiny25/Internal level:
 *   PB1 is controlled through DDRB and PORTB to emulate open-drain behavior.
 */

#define OUT_PIN         PB2
/* Circuit/System level:
 *   Assigns PWM fan-drive output to physical pin 7.
 * ATtiny25/Internal level:
 *   PB2 is toggled by the Timer0 compare-match ISR.
 */

/* ---------- PWM CONFIGURATION ---------- */

#define PWM_TOP_COUNT       255u
/* Circuit/System level:
 *   Defines an 8-bit PWM range from 0 to 255.
 * ATtiny25/Internal level:
 *   Used conceptually as the maximum software PWM phase/duty value.
 */

#define MIN_DUTY_CLAMP      64u
/* Circuit/System level:
 *   Defines the minimum fan-speed clamp as approximately 25% duty.
 *   64 / 255 is approximately 25%.
 * ATtiny25/Internal level:
 *   Used as the minimum 8-bit duty value when clamp is enabled.
 */

#define STARTUP_CYCLES      64u
/* Circuit/System level:
 *   Defines the MIC502-like startup interval as 64 PWM cycles at 100% duty.
 * ATtiny25/Internal level:
 *   Loads an 8-bit counter decremented once per PWM frame in the ISR.
 */

/*
 * Timer0 CTC interrupt frequency:
 *   F_CPU / 8 / (OCR0A + 1)
 *   = 8 MHz / 8 / 39
 *   = 25.641 kHz
 *
 * Software PWM frequency:
 *   25.641 kHz / 256
 *   = about 100.16 Hz
 *
 * Circuit/System level:
 *   This gives a visually smooth PWM frequency for LED testing and a stable control period.
 *
 * ATtiny25/Internal level:
 *   Timer0 compare match interrupt runs once every 39 Timer0 ticks.
 *   The ISR advances an 8-bit software PWM phase from 0 to 255.
 */
#define TIMER0_CTC_TOP      38u
/* Circuit/System level:
 *   Sets the Timer0 interrupt rate used by the software PWM engine.
 * ATtiny25/Internal level:
 *   OCR0A is loaded with 38, so compare match occurs every 39 counts.
 */

/* ---------- GLOBAL STATE ---------- */

volatile uint8_t g_pwm_phase = 0;
/* Circuit/System level:
 *   Tracks the current position inside one PWM cycle.
 *   One full PWM cycle contains 256 phase steps.
 * ATtiny25/Internal level:
 *   Modified inside the Timer0 compare-match ISR.
 *   Declared volatile because it is shared with interrupt context.
 */

volatile uint8_t g_next_duty = 0;
/* Circuit/System level:
 *   Holds the next duty-cycle command calculated by the main control loop.
 *   This prevents duty changes from occurring in the middle of a PWM frame.
 * ATtiny25/Internal level:
 *   Written by main code and copied by the ISR at the frame boundary.
 */

volatile uint8_t g_active_duty = 0;
/* Circuit/System level:
 *   Holds the duty cycle currently being used for the live PWM output.
 * ATtiny25/Internal level:
 *   Read by the ISR on every PWM phase step to decide whether PB2 is HIGH or LOW.
 */

volatile uint8_t g_startup_cycles = STARTUP_CYCLES;
/* Circuit/System level:
 *   Counts how many forced-100% startup PWM cycles remain.
 * ATtiny25/Internal level:
 *   Decremented once per PWM frame inside the ISR.
 */

volatile uint8_t g_control_step_req = 0;
/* Circuit/System level:
 *   Requests the main control algorithm to run once per PWM frame.
 * ATtiny25/Internal level:
 *   Set by the ISR at the beginning of each new PWM frame.
 */

static uint16_t g_avg_vt1 = 0;
/* Circuit/System level:
 *   Stores a filtered version of the VT1 thermistor voltage.
 * ATtiny25/Internal level:
 *   Holds the IIR-filtered ADC0 result in SRAM.
 */

static bool g_clamp_enabled = true;
/* Circuit/System level:
 *   Stores whether the VT2 minimum fan-speed clamp is enabled.
 *   It starts ON after every power-up.
 * ATtiny25/Internal level:
 *   Boolean firmware latch toggled by the PB0 momentary button.
 */

static bool g_shutdown_state = false;
/* Circuit/System level:
 *   Stores whether the controller is currently in shutdown mode.
 *   Shutdown is only allowed when clamp is OFF.
 * ATtiny25/Internal level:
 *   Boolean state variable used by the main control state logic.
 */

static bool g_otf_active = false;
/* Circuit/System level:
 *   Stores whether the overtemperature fault condition is active.
 * ATtiny25/Internal level:
 *   Boolean state variable controlling PB1 open-drain-style output.
 */

static bool g_last_button_state = false;
/* Circuit/System level:
 *   Remembers the previous VT2 button level for edge detection.
 * ATtiny25/Internal level:
 *   Stored PB0 logic state from the previous control step.
 */

static uint8_t g_button_debounce = 0;
/* Circuit/System level:
 *   Prevents one mechanical button press from toggling the clamp multiple times.
 * ATtiny25/Internal level:
 *   Countdown value decremented once per control step.
 */

/* ---------- I/O HELPERS ---------- */

static void otf_release(void) {
    PORTB &= (uint8_t)~(1u << OTF_PIN);
    /* Circuit/System level:
     *   Ensures the /OTF output is not driven HIGH.
     *   This is required for open-drain/open-collector-style behavior.
     * ATtiny25/Internal level:
     *   Clears the PB1 PORT latch.
     *   If PB1 is input, this also disables its internal pull-up.
     */

    DDRB &= (uint8_t)~(1u << OTF_PIN);
    /* Circuit/System level:
     *   Releases /OTF so an external pull-up or LED network can define the inactive state.
     * ATtiny25/Internal level:
     *   Clears the PB1 DDR bit, making PB1 high impedance.
     */
}

static void otf_assert_low(void) {
    PORTB &= (uint8_t)~(1u << OTF_PIN);
    /* Circuit/System level:
     *   Prepares /OTF to actively pull the fault line LOW.
     * ATtiny25/Internal level:
     *   Clears the PB1 output latch so that output mode will drive LOW.
     */

    DDRB |= (uint8_t)(1u << OTF_PIN);
    /* Circuit/System level:
     *   Asserts /OTF active-low.
     *   With VCC → resistor → LED → PB1 wiring, this turns the red LED ON.
     * ATtiny25/Internal level:
     *   Sets PB1 as output, so PB1 sinks current to ground.
     */
}

static void out_enable(void) {
    DDRB |= (uint8_t)(1u << OUT_PIN);
    /* Circuit/System level:
     *   Enables the PWM output driver on physical pin 7.
     * ATtiny25/Internal level:
     *   Sets the PB2 DDR bit, configuring PB2 as a digital output.
     */
}

static void out_force_low(void) {
    g_next_duty = 0;
    /* Circuit/System level:
     *   Commands the PWM output to become 0% duty on the next PWM frame.
     * ATtiny25/Internal level:
     *   Writes zero into the double-buffered duty variable.
     */

    PORTB &= (uint8_t)~(1u << OUT_PIN);
    /* Circuit/System level:
     *   Immediately drives the PWM output LOW.
     *   This is used during shutdown or initialization.
     * ATtiny25/Internal level:
     *   Clears the PB2 PORT latch.
     */
}

static bool vt2_button_high(void) {
    return (PINB & (1u << VT2_BUTTON_PIN)) != 0;
    /* Circuit/System level:
     *   Returns true when the VT2 clamp button drives physical pin 5 HIGH.
     * ATtiny25/Internal level:
     *   Reads the PB0 bit from the PINB input register.
     */
}

/* ---------- ADC ---------- */

static void adc_init(void) {
    ADMUX = CH_VT1;
    /* Circuit/System level:
     *   Selects VT1 as the analog signal to be measured.
     * ATtiny25/Internal level:
     *   Sets the ADC multiplexer to ADC0.
     */

    ADCSRA = (1u << ADEN) | (1u << ADPS2) | (1u << ADPS1);
    /* Circuit/System level:
     *   Turns on the ADC block so the firmware can measure VT1.
     * ATtiny25/Internal level:
     *   ADEN enables the ADC.
     *   ADPS2 and ADPS1 select ADC prescaler = 64.
     *   At F_CPU = 8 MHz, ADC clock is 125 kHz.
     */

#ifdef ADC0D
    DIDR0 |= (1u << ADC0D);
    /* Circuit/System level:
     *   Disables the unused digital input buffer on the analog VT1 pin.
     *   This can reduce input noise and unnecessary current.
     * ATtiny25/Internal level:
     *   Sets the ADC0D bit in DIDR0.
     */
#endif
}

static uint16_t adc_read_channel(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0u) | (ch & 0x0Fu);
    /* Circuit/System level:
     *   Selects which analog input channel is being sampled.
     * ATtiny25/Internal level:
     *   Preserves the upper ADMUX configuration bits and updates the lower MUX bits.
     */

    ADCSRA |= (1u << ADSC);
    /* Circuit/System level:
     *   Starts a dummy ADC conversion after selecting the channel.
     *   This helps settle the ADC multiplexer and sampling capacitor.
     * ATtiny25/Internal level:
     *   Sets the ADSC bit to begin conversion.
     */

    while (ADCSRA & (1u << ADSC)) {
    }
    /* Circuit/System level:
     *   Waits until the dummy ADC conversion is complete.
     * ATtiny25/Internal level:
     *   Polls the ADSC bit until hardware clears it.
     */

    (void)ADC;
    /* Circuit/System level:
     *   Discards the dummy conversion result.
     * ATtiny25/Internal level:
     *   Reads the ADC data register result without using it.
     */

    ADCSRA |= (1u << ADSC);
    /* Circuit/System level:
     *   Starts the real ADC conversion used by the control algorithm.
     * ATtiny25/Internal level:
     *   Sets ADSC again for a second conversion.
     */

    while (ADCSRA & (1u << ADSC)) {
    }
    /* Circuit/System level:
     *   Waits until the real ADC conversion is complete.
     * ATtiny25/Internal level:
     *   Polls ADSC until conversion hardware clears the bit.
     */

    return ADC;
    /* Circuit/System level:
     *   Returns the measured analog voltage as a 10-bit ADC value.
     * ATtiny25/Internal level:
     *   Reads the combined ADCL/ADCH ADC result.
     */
}

static void adc_update_filter(uint8_t ch, uint16_t *avg_store) {
    uint16_t raw = adc_read_channel(ch);
    /* Circuit/System level:
     *   Takes a fresh sample from the selected analog input.
     * ATtiny25/Internal level:
     *   Calls the ADC conversion function and stores the raw 10-bit result.
     */

    if (*avg_store == 0u) {
        *avg_store = raw;
        /* Circuit/System level:
         *   Initializes the filter output with the first valid ADC reading.
         * ATtiny25/Internal level:
         *   Writes the raw ADC value into the filter storage variable.
         */
    } else {
        *avg_store = (uint16_t)(((*avg_store) * 3u + raw) >> 2);
        /* Circuit/System level:
         *   Smooths VT1 changes and reduces small ADC noise.
         *   New filtered value = 75% old value + 25% new sample.
         * ATtiny25/Internal level:
         *   Performs integer IIR filtering using multiply-add and right shift by 2.
         */
    }
}

/* ---------- MATH ---------- */

static uint8_t map_vt1_to_duty(uint16_t vt1_adc) {
    if (vt1_adc <= ADC_VPWM_MIN) {
        return 0u;
        /* Circuit/System level:
         *   VT1 is below the MIC502-style 30% VDD control threshold.
         *   Requested fan duty is 0%.
         * ATtiny25/Internal level:
         *   Returns 8-bit duty value 0.
         */
    }

    if (vt1_adc >= ADC_VPWM_MAX) {
        return 255u;
        /* Circuit/System level:
         *   VT1 is above the MIC502-style 70% VDD control threshold.
         *   Requested fan duty is 100%.
         * ATtiny25/Internal level:
         *   Returns 8-bit duty value 255.
         */
    }

    return (uint8_t)(((uint32_t)(vt1_adc - ADC_VPWM_MIN) * 255u) / ADC_VPWM_SPAN);
    /* Circuit/System level:
     *   Linearly maps VT1 between 30% and 70% of VDD into 0% to 100% PWM duty.
     * ATtiny25/Internal level:
     *   Uses 32-bit arithmetic during multiplication to avoid overflow.
     *   Converts the result back to an 8-bit duty value from 0 to 255.
     */
}

/* ---------- BUTTON / CLAMP ---------- */

static void update_clamp_button(void) {
    bool current = vt2_button_high();
    /* Circuit/System level:
     *   Samples the VT2 clamp button input on physical pin 5.
     * ATtiny25/Internal level:
     *   Reads PB0 through PINB and stores the logic state.
     */

    if (g_button_debounce > 0u) {
        g_button_debounce--;
        /* Circuit/System level:
         *   Continues the debounce lockout after a button press.
         * ATtiny25/Internal level:
         *   Decrements an 8-bit countdown once per control update.
         */
    }

    if (current && !g_last_button_state && (g_button_debounce == 0u)) {
        g_clamp_enabled = !g_clamp_enabled;
        /* Circuit/System level:
         *   A new valid button press toggles the minimum fan-speed clamp.
         *   If it was ON, it becomes OFF. If it was OFF, it becomes ON.
         * ATtiny25/Internal level:
         *   Flips the boolean firmware latch stored in SRAM.
         */

        g_button_debounce = 5u;
        /* Circuit/System level:
         *   Starts an approximately 50 ms debounce window at a 100 Hz control rate.
         * ATtiny25/Internal level:
         *   Loads the debounce counter with 5 control cycles.
         */

        if (g_clamp_enabled && g_shutdown_state) {
            g_shutdown_state = false;
            /* Circuit/System level:
             *   If the user turns clamp ON while the controller was shut down,
             *   shutdown is cancelled because clamp ON means the fan should remain running.
             * ATtiny25/Internal level:
             *   Clears the shutdown-state boolean.
             */

            g_startup_cycles = STARTUP_CYCLES;
            /* Circuit/System level:
             *   Forces a new startup interval after leaving shutdown.
             * ATtiny25/Internal level:
             *   Reloads the startup-cycle counter used by the PWM ISR.
             */

            out_enable();
            /* Circuit/System level:
             *   Re-enables the PWM output driver.
             * ATtiny25/Internal level:
             *   Sets PB2 as output.
             */
        }
    }

    g_last_button_state = current;
    /* Circuit/System level:
     *   Stores the current button state so the next control step can detect a new press.
     * ATtiny25/Internal level:
     *   Updates the edge-detection state variable.
     */
}

/* ---------- CONTROL LOGIC ---------- */

static void update_otf(uint16_t vt1_adc) {
    if (vt1_adc >= ADC_OTF) {
        g_otf_active = true;
        /* Circuit/System level:
         *   VT1 exceeds the overtemperature threshold.
         *   The /OTF fault output should be asserted.
         * ATtiny25/Internal level:
         *   Sets the overtemperature state variable.
         */
    } else {
        g_otf_active = false;
        /* Circuit/System level:
         *   VT1 is below the overtemperature threshold.
         *   The /OTF fault output should be released.
         * ATtiny25/Internal level:
         *   Clears the overtemperature state variable.
         */
    }

    if (g_otf_active) {
        otf_assert_low();
        /* Circuit/System level:
         *   Pulls /OTF LOW and turns on the red LED/fault indicator.
         * ATtiny25/Internal level:
         *   Drives PB1 as output LOW.
         */
    } else {
        otf_release();
        /* Circuit/System level:
         *   Releases /OTF to high impedance and turns off the red LED/fault indicator.
         * ATtiny25/Internal level:
         *   Configures PB1 as input/high-Z.
         */
    }
}

static void mic502_control_step(void) {
    adc_update_filter(CH_VT1, &g_avg_vt1);
    /* Circuit/System level:
     *   Samples and filters the VT1 thermistor-conditioned voltage.
     * ATtiny25/Internal level:
     *   Reads ADC0 and updates the IIR filter variable.
     */

    uint16_t vt1 = g_avg_vt1;
    /* Circuit/System level:
     *   Uses the filtered VT1 value for duty, fault, and shutdown decisions.
     * ATtiny25/Internal level:
     *   Copies the global filtered ADC value into a local variable.
     */

    update_clamp_button();
    /* Circuit/System level:
     *   Updates the VT2 clamp ON/OFF state from the physical button.
     * ATtiny25/Internal level:
     *   Reads PB0, performs edge detection, and applies debounce.
     */

    if (!g_clamp_enabled) {
        /* Circuit/System level:
         *   Shutdown/reset behavior is only allowed when minimum fan-speed clamp is OFF.
         * ATtiny25/Internal level:
         *   Tests the clamp state before executing shutdown logic.
         */

        if (!g_shutdown_state && (vt1 <= ADC_VT1_VIL)) {
            g_shutdown_state = true;
            /* Circuit/System level:
             *   VT1 has fallen below the shutdown threshold, so the controller enters shutdown.
             * ATtiny25/Internal level:
             *   Sets the shutdown state flag.
             */

            g_startup_cycles = 0u;
            /* Circuit/System level:
             *   Cancels any remaining startup interval during shutdown.
             * ATtiny25/Internal level:
             *   Clears the ISR startup-cycle counter.
             */

            out_force_low();
            /* Circuit/System level:
             *   Forces the fan-drive PWM output off.
             * ATtiny25/Internal level:
             *   Clears PB2 and sets next duty to 0.
             */

            otf_release();
            /* Circuit/System level:
             *   Releases /OTF during shutdown.
             * ATtiny25/Internal level:
             *   Sets PB1 high impedance.
             */

            return;
            /* Circuit/System level:
             *   Stops this control step because shutdown overrides normal PWM control.
             * ATtiny25/Internal level:
             *   Returns from mic502_control_step().
             */
        }

        if (g_shutdown_state) {
            if (vt1 >= ADC_VT1_VIH) {
                g_shutdown_state = false;
                /* Circuit/System level:
                 *   VT1 has recovered above the restart threshold, so shutdown ends.
                 * ATtiny25/Internal level:
                 *   Clears the shutdown state flag.
                 */

                g_startup_cycles = STARTUP_CYCLES;
                /* Circuit/System level:
                 *   Restarts the fan with a 64-cycle full-duty startup interval.
                 * ATtiny25/Internal level:
                 *   Reloads the startup-cycle counter for the ISR.
                 */

                out_enable();
                /* Circuit/System level:
                 *   Re-enables PWM output on physical pin 7.
                 * ATtiny25/Internal level:
                 *   Configures PB2 as output.
                 */

                otf_release();
                /* Circuit/System level:
                 *   Releases /OTF at restart.
                 * ATtiny25/Internal level:
                 *   Configures PB1 as input/high-Z.
                 */
            } else {
                out_force_low();
                /* Circuit/System level:
                 *   Keeps the PWM output off while VT1 is still below restart threshold.
                 * ATtiny25/Internal level:
                 *   Keeps PB2 low and next duty at 0.
                 */

                otf_release();
                /* Circuit/System level:
                 *   Keeps /OTF inactive during shutdown.
                 * ATtiny25/Internal level:
                 *   Keeps PB1 high impedance.
                 */

                return;
                /* Circuit/System level:
                 *   Stops this control step because the controller remains shut down.
                 * ATtiny25/Internal level:
                 *   Returns from mic502_control_step().
                 */
            }
        }
    } else {
        if (g_shutdown_state) {
            g_shutdown_state = false;
            /* Circuit/System level:
             *   Clamp ON overrides shutdown state.
             *   The system should return to minimum-speed operation.
             * ATtiny25/Internal level:
             *   Clears the shutdown state flag.
             */

            g_startup_cycles = STARTUP_CYCLES;
            /* Circuit/System level:
             *   Forces a new startup interval when leaving shutdown due to clamp ON.
             * ATtiny25/Internal level:
             *   Reloads the ISR startup-cycle counter.
             */

            out_enable();
            /* Circuit/System level:
             *   Re-enables the PWM output pin.
             * ATtiny25/Internal level:
             *   Sets PB2 as output.
             */
        }
    }

    update_otf(vt1);
    /* Circuit/System level:
     *   Updates the active-low overtemperature fault output based on VT1.
     * ATtiny25/Internal level:
     *   Drives or releases PB1 through helper functions.
     */

    uint8_t duty = map_vt1_to_duty(vt1);
    /* Circuit/System level:
     *   Converts the filtered VT1 voltage into a requested PWM duty.
     * ATtiny25/Internal level:
     *   Produces an 8-bit duty command from 0 to 255.
     */

    if (g_clamp_enabled && (duty < MIN_DUTY_CLAMP)) {
        duty = MIN_DUTY_CLAMP;
        /* Circuit/System level:
         *   When clamp is ON, fan duty is not allowed to fall below 25%.
         * ATtiny25/Internal level:
         *   Replaces the computed duty with 64 if the computed value is lower.
         */
    }

    out_enable();
    /* Circuit/System level:
     *   Ensures the fan-drive output pin is configured as an active output.
     * ATtiny25/Internal level:
     *   Sets PB2 direction bit in DDRB.
     */

    g_next_duty = duty;
    /* Circuit/System level:
     *   Sends the new duty command to the PWM engine.
     * ATtiny25/Internal level:
     *   Writes the double-buffer variable that the ISR will latch at the next PWM frame.
     */
}

/* ---------- TIMER0 ISR PWM ENGINE ---------- */

ISR(TIMER0_COMPA_vect) {
    g_pwm_phase++;
    /* Circuit/System level:
     *   Advances the PWM waveform by one timing step.
     * ATtiny25/Internal level:
     *   Increments an 8-bit phase counter every Timer0 compare interrupt.
     */

    if (g_pwm_phase == 0u) {
        uint8_t duty_snapshot = g_next_duty;
        /* Circuit/System level:
         *   At the start of a new PWM frame, copy the latest requested duty.
         * ATtiny25/Internal level:
         *   Reads the volatile duty buffer into a local ISR variable.
         */

        if (g_startup_cycles > 0u) {
            duty_snapshot = 255u;
            /* Circuit/System level:
             *   During startup, override normal duty and force 100% output.
             * ATtiny25/Internal level:
             *   Sets the ISR duty snapshot to full-scale 255.
             */

            g_startup_cycles--;
            /* Circuit/System level:
             *   Consumes one of the 64 startup PWM cycles.
             * ATtiny25/Internal level:
             *   Decrements the volatile startup counter once per PWM frame.
             */
        }

        g_active_duty = duty_snapshot;
        /* Circuit/System level:
         *   Makes the selected duty active for the next full PWM frame.
         * ATtiny25/Internal level:
         *   Stores the latched duty for use in ISR phase comparisons.
         */

        g_control_step_req = 1u;
        /* Circuit/System level:
         *   Requests the main loop to run the control algorithm once per PWM frame.
         * ATtiny25/Internal level:
         *   Sets a volatile flag checked in main().
         */
    }

    if (g_active_duty == 0u) {
        PORTB &= (uint8_t)~(1u << OUT_PIN);
        /* Circuit/System level:
         *   0% duty keeps the PWM output LOW for the entire frame.
         * ATtiny25/Internal level:
         *   Clears PB2 in PORTB.
         */
    } else if (g_active_duty == 255u) {
        PORTB |= (uint8_t)(1u << OUT_PIN);
        /* Circuit/System level:
         *   100% duty keeps the PWM output HIGH for the entire frame.
         * ATtiny25/Internal level:
         *   Sets PB2 in PORTB.
         */
    } else if (g_pwm_phase < g_active_duty) {
        PORTB |= (uint8_t)(1u << OUT_PIN);
        /* Circuit/System level:
         *   PWM output is in the ON portion of the duty cycle.
         * ATtiny25/Internal level:
         *   Sets PB2 HIGH while phase is less than active duty.
         */
    } else {
        PORTB &= (uint8_t)~(1u << OUT_PIN);
        /* Circuit/System level:
         *   PWM output is in the OFF portion of the duty cycle.
         * ATtiny25/Internal level:
         *   Clears PB2 LOW when phase reaches or exceeds active duty.
         */
    }
}

/* ---------- HARDWARE INIT ---------- */

static void hw_init(void) {
    cli();
    /* Circuit/System level:
     *   Prevents interrupts from running while hardware registers are being configured.
     * ATtiny25/Internal level:
     *   Clears the global interrupt enable bit.
     */

    out_enable();
    /* Circuit/System level:
     *   Configures physical pin 7 as the PWM output pin.
     * ATtiny25/Internal level:
     *   Sets PB2 as output.
     */

    out_force_low();
    /* Circuit/System level:
     *   Starts the fan-drive output in the OFF state.
     * ATtiny25/Internal level:
     *   Clears PB2 and sets next duty to 0.
     */

    DDRB &= (uint8_t)~(1u << VT2_BUTTON_PIN);
    /* Circuit/System level:
     *   Configures physical pin 5 as the VT2 clamp button input.
     * ATtiny25/Internal level:
     *   Clears the PB0 direction bit so PB0 is input.
     */

    PORTB &= (uint8_t)~(1u << VT2_BUTTON_PIN);
    /* Circuit/System level:
     *   Disables internal pull-up because the external circuit provides a 10 kΩ pulldown.
     * ATtiny25/Internal level:
     *   Clears the PB0 PORT latch while PB0 is configured as input.
     */

    otf_release();
    /* Circuit/System level:
     *   Initializes /OTF as inactive/floating.
     * ATtiny25/Internal level:
     *   Configures PB1 as high impedance.
     */

    adc_init();
    /* Circuit/System level:
     *   Initializes VT1 analog measurement.
     * ATtiny25/Internal level:
     *   Enables ADC0 and disables the ADC0 digital input buffer.
     */

    TCCR0A = (1u << WGM01);
    /* Circuit/System level:
     *   Configures Timer0 to generate a fixed timing interrupt for software PWM.
     * ATtiny25/Internal level:
     *   Sets Timer0 CTC mode using OCR0A as the compare value.
     */

    TCCR0B = (1u << CS01);
    /* Circuit/System level:
     *   Starts Timer0 with a divide-by-8 prescaler.
     * ATtiny25/Internal level:
     *   Sets the CS01 bit in TCCR0B.
     */

    OCR0A = TIMER0_CTC_TOP;
    /* Circuit/System level:
     *   Sets the Timer0 compare interval that determines the PWM phase-step rate.
     * ATtiny25/Internal level:
     *   Loads OCR0A with 38, giving compare match every 39 timer counts.
     */

    TIMSK |= (1u << OCIE0A);
    /* Circuit/System level:
     *   Enables the Timer0 compare-match interrupt used by the PWM engine.
     * ATtiny25/Internal level:
     *   Sets the OCIE0A bit in the Timer Interrupt Mask register.
     */

    set_sleep_mode(SLEEP_MODE_IDLE);
    /* Circuit/System level:
     *   Allows the CPU to sleep between control/PWM events while timers remain active.
     * ATtiny25/Internal level:
     *   Selects idle sleep mode.
     */

    sei();
    /* Circuit/System level:
     *   Enables the interrupt-driven PWM engine.
     * ATtiny25/Internal level:
     *   Sets the global interrupt enable bit.
     */
}

/* ---------- MAIN ---------- */

int main(void) {
    hw_init();
    /* Circuit/System level:
     *   Initializes all hardware resources needed for the fan controller.
     * ATtiny25/Internal level:
     *   Configures GPIO, ADC, Timer0, interrupts, sleep mode, and initial output states.
     */

    g_clamp_enabled = true;
    /* Circuit/System level:
     *   Every power-up starts with minimum fan-speed clamp ON.
     * ATtiny25/Internal level:
     *   Initializes the clamp firmware latch to true.
     */

    g_shutdown_state = false;
    /* Circuit/System level:
     *   Controller starts in non-shutdown state.
     * ATtiny25/Internal level:
     *   Clears the shutdown state flag.
     */

    g_startup_cycles = STARTUP_CYCLES;
    /* Circuit/System level:
     *   Starts the 64-cycle full-duty startup interval after power-up.
     * ATtiny25/Internal level:
     *   Loads the volatile startup counter used by the ISR.
     */

    g_next_duty = MIN_DUTY_CLAMP;
    /* Circuit/System level:
     *   After startup, the first normal duty command is the 25% clamp level.
     * ATtiny25/Internal level:
     *   Writes 64 into the double-buffered duty variable.
     */

    while (1) {
        if (g_control_step_req) {
            g_control_step_req = 0u;
            /* Circuit/System level:
             *   A new PWM frame has started, so this control request is now being serviced.
             * ATtiny25/Internal level:
             *   Clears the volatile flag set by the Timer0 ISR.
             */

            mic502_control_step();
            /* Circuit/System level:
             *   Runs one full fan-control update:
             *   VT1 sampling, button handling, shutdown/reset logic, /OTF update, and duty update.
             * ATtiny25/Internal level:
             *   Executes the control function in main context rather than inside the ISR.
             */
        }

        sleep_mode();
        /* Circuit/System level:
         *   Saves power and keeps the CPU idle until the next Timer0 interrupt.
         * ATtiny25/Internal level:
         *   Enters idle sleep mode; Timer0 interrupt wakes the CPU.
         */
    }
}
