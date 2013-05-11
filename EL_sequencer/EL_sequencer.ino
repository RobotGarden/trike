/** Sketch for Robot Garden Maker Faire 2013 Trike with
 *  Smoke ring cannon and blink.
 * @author Daniel Casner
 *  Based on code by Mike Grusin, SparkFun Electronics
 */

#include <Bounce.h>

/// The EL channels are on pins 2 through 9
#define A 2
#define B 3
#define C 4
#define D 5
#define E 6
#define F 7
#define G 8
#define H 9
/// Status LED
#define STATUS 13
/// Battery monitor ADC
#define BATTERY 7
#define LOOP_PERIOD 3900 //us ~ 256Hz

Bounce toggle = Bounce(20, A0);

float battery_scale = 20.0; ///< Battery scale factor for ADC. TODO, replace with real value
float cur_bat;
int bar_graph[] = {10, 11, 12};
byte loop_cnt, ramp, el_phase, phase_cnt;
int phaser[] = {A, B, C};

void setup() {                
  // Initialize the pins as outputs

  int i;
  for (i=A; i<=H; i++) {
       pinMode(i, OUTPUT);
       digitalWrite(i, 0);
  }
  /// Status pin on 
  pinMode(STATUS, OUTPUT);
  /// Use the internal 1.1V analog reference.

  analogReference(INTERNAL);
  /// Setup toggle button
  pinMode(A0, INPUT_PULLUP);
  /// bling ramp
  ramp = 0;
  /// EL sequencer phase
}

void loop() 
{  

  byte i;

  unsigned long loop_time, lstart;

  lstart = micros();

  loop_cnt++

  // battery monitor
  cur_bat = ((float)analogRead(BATTERY))*battery_scale;
  digitalWrite(bar_graph[2], cur_bat > 11.9f);

  digitalWrite(bar_graph[1], cur_bat > 11.7f);
  if (cur_bat > 11.5f) digitalWrite(bar_graph[0], 1);

  else digitalWrite(bar_graph[0], loop_cnt > 128);


  if (cur-bat > 11.5f) { // Good battery
     digitalWrite(H, 1); // Always on if battery is good

     if (toggle.update() && toggle.read()) {
        if (ramp == 0) ramp = 250;
        else ramp = 0;
     }

     if (ramp > 0) {
       for (i=0; i<3; i++) {
           digitalWrite(phaser[i], i==el_phase);
       }
       if (phase_cnt++ > ramp) {
         if (++el_phase > 2) el_phase = 0;
         if (ramp > 25) ramp -= 2;
         phase_cnt = 0;
       }
     }
     else { // If not ramping
       for(i=0; i<3; i++) { // Turn them all on
         digitalWrite(phaser[i], 1); 
       }
     }
  }
  else { // Battery is near dead shut things down
    for (i=A; i<=H; i++) digitalWrite(i, 0);
  }

  loop_time = micros() - lstart;

  if (loop_time > LOOP_PERIOD) { // Taking too long to execute
    digitalWrite(STATUS, 1);
  else {
    digitalWrite(STATUS, loop_cnt > 128);
    delayMicroseconds(LOOP_PERIOD - loop_time);
  }
}
