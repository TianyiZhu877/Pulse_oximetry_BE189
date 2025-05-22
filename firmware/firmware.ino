#include <util/atomic.h>
#define RED   0
#define IR    1

// pin definition
const int sample_pin = A0;
const int red_led_pin = 3;
const int ir_led_pin = 2;

// constant for sampling
const uint8_t prescaler = 64;
const uint16_t base_f = 80;
const uint8_t red_divider = 1;
const uint8_t ir_divider = 1;
uint8_t dividers[2] = {red_divider, ir_divider};
const float CPU_F = 16e6;


// global variables for ISR
const int isr_period = red_divider * ir_divider * 2;
uint16_t isr_count = 0;
// global variables for loop
long last_red_sample  = 0;
long last_ir_sample  = 0;

// shared buffer sampling timer isr and loop
volatile long ir_t;
volatile int ir_v;
volatile long red_t;
volatile int red_v;

inline void sample(uint8_t led) {
  if (led == RED) {
    red_v  = analogRead(sample_pin);
    red_t = millis();
  } else if (led == IR) {
    ir_v  = analogRead(sample_pin);
    ir_t = millis();
  }
}

inline void switching(uint8_t led) {
  if (led == RED) {
    digitalWrite(red_led_pin, HIGH);
    digitalWrite(ir_led_pin, LOW);
  } else if (led == IR) {
    digitalWrite(ir_led_pin, HIGH);
    digitalWrite(red_led_pin, LOW);
  }
}


// sampling ISR
ISR(TIMER1_COMPA_vect) {
  uint8_t sampling_led = isr_count%2;
  uint8_t switching_led = (isr_count+1)%2;
  int   sample_count = isr_count/2;
  int   switch_count = (isr_count+1)/2;
   
  if (sample_count % dividers[sampling_led] == 0) {
    sample(sampling_led);
  }
  if (switch_count % dividers[switching_led] == 0) {
    switching(switching_led);
  }
  
  isr_count += 1;
  if (isr_count == isr_period) 
    isr_count = 0;
}



void setup() {
  Serial.begin(115200);
  
  float ocr1a_f = CPU_F / prescaler / base_f / 2 - 1;
//  Serial.print("ocr1a: ");
//  Serial.println((uint16_t)ocr1a_f);

  pinMode(red_led_pin, OUTPUT); 
  pinMode(ir_led_pin, OUTPUT); 

  // put your setup code here, to run once:
  noInterrupts();           // Disable interrupts during config

  TCCR1A = 0;               // Clear control register A
  TCCR1B = 0;               // Clear control register B
  TCNT1  = 0;               // Initialize counter value

  OCR1A = (uint16_t)ocr1a_f;              // Compare match value (zero-indexed)

  // CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);

  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Enable Timer1 Compare Match A interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();             // Enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
  long ir_t_local, red_t_local;
  int red_v_local, ir_v_local;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    red_v_local = red_v;
    red_t_local = red_t;
    ir_v_local = ir_v;
    ir_t_local = ir_t;
  }
  
  if ((red_t_local != last_red_sample) or (ir_t_local != last_ir_sample)) {
    last_red_sample = red_t_local;
    last_ir_sample = ir_t_local;
    Serial.print(red_v_local);
    Serial.print(',');
    Serial.println(ir_v_local);
  }
}
