#ifndef _TIMER3_H
#define _TIMER3_H

extern const int MAX_RESULTS;
extern float results_speed[128];
extern float results_e[128];
extern int results_speed_index;
extern int results_e_index;
extern float speed;
extern float e;
// Global variable to remember the
// on/off state of the LED.
volatile boolean DEBUG_LED_STATE = false;
//// For a blinking experiment,
//// remove "volatile" doesn't affact the result.

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR(TIMER3_COMPA_vect) {

  if (results_speed_index < MAX_RESULTS) {
    // your data capture routine.
    results_speed[results_speed_index] = speed;  // speed
    results_speed_index++;
  }
  if (results_e_index < MAX_RESULTS) {
    // your data capture routine.
    results_e[results_e_index] = e;  // line_error
    results_e_index++;
  }

  // Invert LED state.
  DEBUG_LED_STATE = !DEBUG_LED_STATE;

  // Enable/disable LED
  digitalWrite(13, DEBUG_LED_STATE);
}

void setupTimer3() {
  /*
 * OCR3A stores the value to count up to, using 16 binary bits, 
 in order to determine when to stop the timer and run the ISR.
 * TCCR3B stores the prescaler value, set by a combination of 3 
 individual bits, which determines how quickly your timer will 
 be counting up as a division of the CPU clock.

Therefore, our setup can be described in two steps as:

  what value to count up to (OCR3A)
  how quickly the timer counts (TCCR3B)

 * When the Timer3 count reaches the match value, the ISR is run, 
and timer3 resets its own count to zero. It will count up toward 
OCR3A again.

 */
  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;  // set entire TCCR3A register to 0
  TCCR3B = 0;  // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  /*
   * TCCR3B = Store into TCCR3B...
   * TCCR3B | ...the current value of TCCR3B Logically OR'd with...
   * ( 1 << WGM32 ) ...a 1 shifted across to the location of WGM32.
Where:

(1 << WGM32) means shift a 1 up to the location of WGM32.
We use the logical OR operation (the | symbol) because it maintains 
any other values in the register.
   */

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS30) | (1 << CS32);


  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 1024
  // Timer freq = 16000000/1024 = 15625
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 15625 / 10 (we desire 10 hz,i.e.,0.1 sec/circle)
  // OCR3A = 312.5;
  OCR3A = 3906.25;  // 0.25 second

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}
#endif