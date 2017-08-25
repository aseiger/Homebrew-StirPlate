#include <PinChangeInt.h>

#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_LEDBackpack.h>
#include <Wire.h>

#include <PID_v1.h>

#define DEBUG_LED         13
#define PIN_ENCODER_A      4
#define PIN_ENCODER_B      5
#define PIN_ENCODER_BUTTON 6
#define TRINKET_PINx       PIND
#define ENCODER_INTERRUPT_PIN 4
#define RPM_INTERRUPT_PIN 3
#define HALTPIN 10

#define MOTOR_PWM_PIN 11

#define UPDATE_RATE 100
#define HIST_DEPTH 1
//construct the display instance
Adafruit_AlphaNum4 display = Adafruit_AlphaNum4();

volatile int state = LOW;
double m_power = 0;
double perc_m_power = 0;
volatile unsigned long rpm_ticks = 0;
volatile unsigned long timeof_last_int_tick;
volatile unsigned long int_tick_time;

volatile unsigned long timeof_last_spin;


volatile int tick_watchdog = 0;
unsigned long watchdog_last_time = 0;
unsigned long max_tick_time = 300;

int last_tick_count = 0;

int num_ticks_per_check = 1;


unsigned long timeLastUpdated = 0;
volatile double rpm = 0.0;
double pid_rpm;
volatile double rpm_hist[HIST_DEPTH];
double desired_rpm = 50;

//Specify the links and initial tuning parameters
PID myPID(&pid_rpm, &m_power, &desired_rpm, 2, 0.05, 0, DIRECT);

void clearDisplay(void);

void setup() {
  // put your setup code here, to run once:
  //GPIO initialization
  pinMode(DEBUG_LED, OUTPUT);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(RPM_INTERRUPT_PIN, INPUT);
  TCCR2B = TCCR2B & 0b11111000 | 0x07;
  //attachInterrupt(digitalPinToInterrupt(ENCODER_INTERRUPT_PIN), encoder_ISR, RISING);
  PCintPort::attachInterrupt(ENCODER_INTERRUPT_PIN, encoder_ISR, RISING); // attach a PinChange Interrupt to our pin on the rising edge
  attachInterrupt(digitalPinToInterrupt(RPM_INTERRUPT_PIN), RPM_ISR, RISING);
  
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);
  pinMode(HALTPIN, OUTPUT);
  digitalWrite(HALTPIN, HIGH);
  //start the display up
  display.begin(0x70);
  //clear the display
  clearDisplay();
  //Serial.begin(9600);
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 255);

  //turn the PID on
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  if (!digitalRead(PIN_ENCODER_BUTTON)) {
    //m_power = 0;
    desired_rpm = 50;
  }

  char displayBuffer[15];
  int strsize = 0;
  //Serial.println(UPDATE_RATE);
  if (millis() > (timeLastUpdated + UPDATE_RATE)) {
    
    //state = !state;
    timeLastUpdated = millis();
    if (desired_rpm > 50) {
      digitalWrite(HALTPIN, HIGH);
      myPID.SetMode(AUTOMATIC);
      myPID.Compute();
      //write power to motor
      analogWrite(MOTOR_PWM_PIN, m_power);
      //analogWrite(DEBUG_LED, m_power);
      strsize = sprintf(&displayBuffer[0], "%4i", (int)desired_rpm);
    } else {
      myPID.SetMode(MANUAL);
      m_power = 0.0;
      analogWrite(MOTOR_PWM_PIN, m_power);
      //analogWrite(DEBUG_LED, m_power);
      digitalWrite(HALTPIN, LOW);
      strsize = sprintf(&displayBuffer[0], "HALT");
    }
    if (strsize <= 5) {
      display.writeDigitAscii(0, displayBuffer[0]);
      display.writeDigitAscii(1, displayBuffer[1]);
      display.writeDigitAscii(2, displayBuffer[2]);
      display.writeDigitAscii(3, displayBuffer[3]);
      display.writeDisplay();

      //Serial.print("m_power: ");
      //Serial.print(m_power);
      //Serial.print(" desired RPM: ");
     // Serial.print(desired_rpm);
      //Serial.print(" error :");
     // Serial.println(100*((pid_rpm - desired_rpm)/pid_rpm));
      
    }
  }

  //make sure we catch the zero RPM case
  if ((millis() - watchdog_last_time) > max_tick_time) {
    if (tick_watchdog == 0) {
      rpm = 0;
    }
    tick_watchdog = 0;
    watchdog_last_time = millis();
  }
  pid_rpm = rpm;
}

void clearDisplay(void) {
  display.writeDigitRaw(0, 0x0);
  display.writeDigitRaw(1, 0x0);
  display.writeDigitRaw(2, 0x0);
  display.writeDigitRaw(3, 0x0);
  display.writeDisplay();
}

void encoder_ISR(void) {
  //state = !state;
  // read in the encoder state first
  if(millis() > (timeof_last_spin + 25)){
    if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_B)) {
      //if(m_power != 0) m_power--;
      if (desired_rpm != 50) desired_rpm -= 10;
    } else {
      //m_power++;
      desired_rpm += 10;
    }
    timeof_last_spin = millis();
  }
  
}

void RPM_ISR(void) {
  rpm_ticks++;
  tick_watchdog = 1;
  int_tick_time = micros() - timeof_last_int_tick;
  timeof_last_int_tick = micros();
  //increment the history through the ranks;
  for(int i = HIST_DEPTH-1; i > 0; i--){
    rpm_hist[i] = rpm_hist[i-1];
  }
  //set the t=0 entry
  float rpm_temp = ((20 * 60000) / ((int_tick_time)));
  if(rpm_temp > 0) rpm_hist[0] = rpm_temp;
//  if(((abs(rpm_temp - rpm) > 100) & (rpm > 0)) | (rpm < 0)){
//    //Serial.println(2*rpm);
//    rpm_hist[0] = rpm_hist[0];
//  } else {
//    rpm_hist[0] = rpm_temp;
//  }
  //compute the average of them all (could be a real FIR filter if we had a known RPM)
  rpm = rpm_hist[0];
  for(int i = 1; i < HIST_DEPTH; i++){
    rpm = (rpm + rpm_hist[i])/2;
  }
  //rpm = ((20 * 60000) / ((int_tick_time)));
}

