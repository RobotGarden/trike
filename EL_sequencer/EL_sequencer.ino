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
/// Motor PWM output for bubble rotor
#define ROTOR  10
/// Motor PWM output for bubble blower
#define BLOWER 11
/// Debounced button input
Bounce toggle = Bounce(A4, 5);

/// Main loop target period
#define LOOP_PERIOD 39000 //us ~ 25Hz
/// Battery scale factor for ADC. From manual calibation
float battery_scale = 0.774;
/// Motor shut off value
#define MOTOR_OFF 0
/// Bubble rotor set speed
#define ROTOR_ON 20
/// Bubble blower set speed
#define BLOWER_ON 128

float cur_bat;
byte loop_cnt, ramp, el_phase, phase_cnt;
int phaser[] = {A, B, C};

void setup() {                
  // Initialize the pins as outputs

  int i;
  
  Serial.begin(9600);
  Serial.println("Hello world");
  
  for (i=A; i<=H; i++) {
       pinMode(i, OUTPUT);
       digitalWrite(i, 0);
  }
  /// Status pin on 
  pinMode(STATUS, OUTPUT);
  /// Use the internal 1.1V analog reference.
  analogReference(INTERNAL);
  /// Setup toggle button
  pinMode(A4, INPUT);
  /// PWM setup
  pinMode(ROTOR,  OUTPUT);
  analogWrite(ROTOR, MOTOR_OFF);
  pinMode(BLOWER, OUTPUT);
  analogWrite(BLOWER, MOTOR_OFF);
  /// bling ramp
  ramp = 0;
  el_phase  = 0;
  phase_cnt = 0;
  cur_bat = 12.0f;
}

void loop() 
{  

  byte i;
  float sample;
  unsigned long loop_time, lstart;

  lstart = micros();

  loop_cnt++;
  
  toggle.update();

  // battery monitor
  sample = ((float)analogRead(BATTERY))*battery_scale;
  cur_bat = cur_bat * 0.999 + sample * 0.001; // Low pass filter

  if (cur_bat > 11.5f) { // Good battery
  
    if (cur_bat > 11.7f) { // Normal operation
      digitalWrite(H, 1); // Always on if battery is good
    }
    else { // Time to go charge the battery, indicate something is up
      digitalWrite(H, loop_cnt > 128); // Blink at just under 1Hz
    }  
     
    analogWrite(ROTOR,  ROTOR_ON);
    analogWrite(BLOWER, BLOWER_ON);
     
    if (toggle.risingEdge()) {
      if (ramp == 0) ramp = 50;
      else ramp = 0;
    }

    if (ramp > 0) {
      for (i=0; i<3; i++) {
          digitalWrite(phaser[i], i==el_phase);
      }
      if (phase_cnt++ > ramp) {
        if (++el_phase > 2) el_phase = 0;
        if (ramp > 2) ramp -= 2;
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
    analogWrite(ROTOR, MOTOR_OFF);
    analogWrite(BLOWER, MOTOR_OFF);
  }

  Serial.print(cur_bat);
  Serial.print("\t");
  Serial.print(ramp);
  Serial.print("\t");
  Serial.print(phase_cnt);
  Serial.println("");

  loop_time = micros() - lstart;

  if (loop_time > LOOP_PERIOD) { // Taking too long to execute
    digitalWrite(STATUS, 1);
  }
  else {
    digitalWrite(STATUS, loop_cnt > 128);
    delayMicroseconds(LOOP_PERIOD - loop_time);
  }
}
