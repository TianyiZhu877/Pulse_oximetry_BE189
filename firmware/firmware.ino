#include <util/atomic.h>
#include "filters.h"
#include "types.h"

#define RED   0
#define IR    1

// pin definition
const int sample_pin = A0;
const int red_led_pin = 7;
const int ir_led_pin = 6;

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
long last_red_sample_time  = 0;
long last_ir_sample_time  = 0;
sample_t ir_local, red_local;

// shared buffer sampling timer isr and loop
volatile sample_t ir_sample, red_sample;

// filter classes
DCFilter red_dc_filter;
DCFilter ir_dc_filter;

inline void sample(sample_t& sample) {
  sample.v  = analogRead(sample_pin);
  sample.t = millis();
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

inline void filter(sample_t& sample, DCFilter& dc_filter) {
//  Serial.print("before filtering");
//  Serial.print(sample.v);
  sample.v = dc_filter.step(sample.v);
  sample.dc = dc_filter.get_dc();
//  Serial.print(", after filtering: ");
//  Serial.println(sample.v);
}


// sampling ISR
ISR(TIMER1_COMPA_vect) {
  uint8_t sampling_led = isr_count%2;
  uint8_t switching_led = (isr_count+1)%2;
  int   sample_count = isr_count/2;
  int   switch_count = (isr_count+1)/2;

  sample_t new_sample;
  if (sample_count % dividers[sampling_led] == 0) {
    sample(new_sample);
  }
  if (switch_count % dividers[switching_led] == 0) {
    switching(switching_led);
  }
  if (sample_count % dividers[sampling_led] == 0) {
    if (sampling_led == RED) {
      filter(new_sample, red_dc_filter);
      red_sample = new_sample;
    }
    else if (sampling_led == IR) {
      filter(new_sample, ir_dc_filter);
      ir_sample = new_sample;
    }
  }
  
  isr_count += 1;
  if (isr_count == isr_period) 
    isr_count = 0;
}

void serial_publish_task(bool publish_t_dc = false) {
  
  if (publish_t_dc) {
    Serial.print(red_local.t);
    Serial.print(',');
    Serial.print(red_local.dc);
    Serial.print(',');
  }
  Serial.print(red_local.v);
  Serial.print(',');

  if (publish_t_dc) {
    Serial.print(ir_local.t);
    Serial.print(',');
    Serial.print(ir_local.dc);
    Serial.print(',');
  }
  Serial.print(ir_local.v);
  Serial.print(',');
  
  Serial.println("32, -32");

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
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    red_local.t = red_sample.t;
    red_local.dc = red_sample.dc;
    red_local.v = red_sample.v;
    ir_local.t = ir_sample.t;
    ir_local.dc = ir_sample.dc;
    ir_local.v = ir_sample.v;
  }


  if ((red_local.t != last_red_sample_time) or (ir_local.t != last_red_sample_time)) {
    last_red_sample_time = red_local.t;
    last_ir_sample_time = ir_local.t;
    
    serial_publish_task();
  }
  
}
