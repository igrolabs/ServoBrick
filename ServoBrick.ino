#define DEBUG 

#define PIN_IN_C1 2 // C1 from HUB
#define PIN_IN_C2 3 // C2 from HUB

#define  PIN_OUT_GND 4  // pin to transistor gate
#define  PIN_OUT_C1 5   // pin to IN1 of motor driver
#define  PIN_OUT_C2 6   // pin to IN2 of motor driver

#define BTN_DEBUG 11

#define AVG_ARRAY_SIZE 10             // used to calculate duty cycle as average from N last pulses
#define MAX_CHANGE_TIME_MICROS 10000  // used as pulse timeout for duty cycle calculations for 0 and 100% (continiuous LOW or HIGH)

#define SERVO_ROTATE_TIME_MICROS 200000 // duration for powering servo motor - it should rotate within given timeout, then the power is turned off

#define DELTA_C 0.02  // minimum change in C1/C2 signal from hub that is sent to servo

#define MAX_C_VALUE 1.0 // NOT WORKING YET!! limit for maximum C1/C2 output value

#ifdef DEBUG
  #define DEBUG_PRINT(x)     Serial.print (x)
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
#endif

struct pwmState_t {
  int c;
  int state;
  unsigned long lastChangeTime;

  unsigned long timeChangeHigh;
  unsigned long timeChangeLow;

  int index;
  unsigned long timeHigh[AVG_ARRAY_SIZE];
  unsigned long timeCycle[AVG_ARRAY_SIZE];
};

pwmState_t pwmState[2];

double c1 = 0;
double c2 = 0;
unsigned long servoTimeLimit = 0;
int servo_power_status = 0;

// debug for duty cycle calculation
void print_debug(int c){
  noInterrupts(); 

  pwmState_t *s = &pwmState[c];
  Serial.print("c = "); Serial.println(s->c);
  Serial.print("state = "); Serial.println(s->state);
  Serial.print("lastChangeTime = "); Serial.println(s->lastChangeTime);
  for (int i=0; i<AVG_ARRAY_SIZE; i++){
    Serial.print("timeHigh[i] = "); Serial.println(s->timeHigh[i]);
  }
  for (int i=0; i<AVG_ARRAY_SIZE; i++){
    Serial.print("timeCycle[i] = "); Serial.println(s->timeCycle[i]);
  }

   interrupts();
}

void setup() {
  init_pwm_state(0);
  pinMode(PIN_IN_C1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_C1), interrupt_C1, CHANGE);

  init_pwm_state(1);
  pinMode(PIN_IN_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_C2), interrupt_C2, CHANGE);

  pinMode(PIN_OUT_C1, OUTPUT);
  digitalWrite(PIN_OUT_C1, LOW);

  pinMode(PIN_OUT_C2, OUTPUT);
  digitalWrite(PIN_OUT_C2, LOW);
  
  pinMode(PIN_OUT_GND, OUTPUT);
  digitalWrite(PIN_OUT_GND, LOW);

  pinMode(BTN_DEBUG, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  loop_servo();
}

void loop_servo() {
  // get duty cycle for RC hub control signals
  double new_c1 = calculate_duty(0); 
  double new_c2 = calculate_duty(1); 

  double delta1 = abs(c1 - new_c1);
  double delta2 = abs(c2 - new_c2);

  unsigned long time = micros();

  // check if input signal from HUB has been changed
  if ( (delta1 > DELTA_C) || (delta2 > DELTA_C) ) { 
    // give servo some time to rotate
    servoTimeLimit = time + SERVO_ROTATE_TIME_MICROS;
    digitalWrite(PIN_OUT_GND, HIGH);   // power on servo

    DEBUG_PRINT("rotate");
    DEBUG_PRINT(" new C1 = "); DEBUG_PRINT(new_c1);
    DEBUG_PRINT(" new C2 = "); DEBUG_PRINTLN(new_c2);

    // servo sometimes can't return directly to zero position (usually after sleep)
    if ( new_c1 + new_c2 < DELTA_C ) { // if new position should be ZERO...
      
      DEBUG_PRINT("rotate 1/2 after sleep");
      DEBUG_PRINT(" C1 / 2 = "); DEBUG_PRINT(c1 / 2);
      DEBUG_PRINT(" C2 / 2 = "); DEBUG_PRINTLN(c2 / 2);

      // first move to 1/2 of last known position 
      output_pwm(c1 / 2, c2 / 2);
      // give servo some time
      delay(20);
      // even more time if servo was powered off
      if (servo_power_status == 0)
        delay(40);
      // then finally return to zero position
      output_pwm(0, 0);
    }

    // update signal for servo
    output_pwm(new_c1,new_c2);
    servo_power_status = 1;

    c1 = new_c1;
    c2 = new_c2;

  }

  // power off servo after time limit - this is needed only in intermidiate positions
  if ((time > servoTimeLimit) && (c1 < 1.0) && (c2 < 1.0) && (c1 + c2 > 0))  
  {
    digitalWrite(PIN_OUT_GND, LOW); // power off
    
    if (servo_power_status == 1) 
      DEBUG_PRINTLN("sleep");

    servo_power_status = 0;
  }
  else { // default to power on
    digitalWrite(PIN_OUT_GND, HIGH);   
    servo_power_status = 1;
  }

  if (digitalRead(BTN_DEBUG) == LOW){
    debug_btn();
  }

  delay(20);
}

void debug_btn()
{
  
}

void interrupt_C1() {
  pin_update_pwm(0, PIN_IN_C1);
}

void interrupt_C2() {
  pin_update_pwm(1, PIN_IN_C2);
}

void init_pwm_state(int c){
  pwmState_t *s = &pwmState[c];
  s->c = c;
  s->state = 0;
  s->index = 0;
}

// save pulse timings for duty cycle calculation
void pin_update_pwm(int c, int pin) {
  pwmState_t *s = &pwmState[c];

  int value = digitalRead(pin);

  if (value == s->state) return; //no change!

  unsigned long time = micros();
  int i = s->index;
  
  if (value == HIGH) {
    //save cycle time = time since previous signal change to HIGH
    s->timeCycle[i] = time - s->timeChangeHigh;
    s->timeChangeHigh = time;
  } 

  if (value == LOW) {
    // save duty cycle = time since signal changed to HIGH
    s->timeHigh[i] = time - s->timeChangeHigh;
    s->timeChangeLow = time;

    s->index += 1;
    if (s->index == AVG_ARRAY_SIZE) {
      s->index = 0;
    }
  }

  s->state = value;
  s->lastChangeTime = time;
}

// excludes zeros and out of cycle values (>2500 micros) from average calculation
unsigned long smart_avg(unsigned long *a){

  unsigned long sum = 0;
  int count = 0;

  for (int i=0; i<AVG_ARRAY_SIZE; i++){
    unsigned long val = a[i];

    if (0 < val && val < 2500) {
      sum += val;
      count += 1;
    }
  }

  if (count == 0)
     return 0;

  return sum / count;
}

// calculate duty cycle based on saved timings
double calculate_duty(int c) {
  double duty = 0;
  noInterrupts(); // disable interrupts while calculating, so global vars don't get updated

  pwmState_t *s = &pwmState[c];

  if (s->lastChangeTime < micros() - MAX_CHANGE_TIME_MICROS) {
    // old data, using last state
    if (s->state == 0)
      duty = 0;
    else
      duty = 1;
  } else {
    double avg_cycle = smart_avg(s->timeCycle);
    double avg_high  = smart_avg(s->timeHigh);
    if (avg_cycle > 0) {
      duty = avg_high / avg_cycle;
    } else
    {
      duty = 0;
    }
  }
 
  interrupts(); // enable interrupts again

  return duty;
}

// update motor driver inputs
void output_pwm(double c1, double c2) {
 
  if (c1 > MAX_C_VALUE)
    c1 = MAX_C_VALUE;

  if (c2 > MAX_C_VALUE)
    c2 = MAX_C_VALUE;

 if (c1 > 0) {   
    analogWrite(PIN_OUT_C1, c1 * 255);
  } else {
    digitalWrite(PIN_OUT_C1, LOW);
  }

  if (c2 > 0) {   
    analogWrite(PIN_OUT_C2, c2 * 255);
  } else {
    digitalWrite(PIN_OUT_C2, LOW);
  }
}