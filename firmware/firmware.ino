#include <util/atomic.h>
#include "filters.h"
#include "types.h"
#include "lcd.h"

#define RED   0
#define IR    1

const bool inverted = true;
const int analog_max = 1023;

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
uint32_t last_red_sample_time  = 0;
uint32_t last_ir_sample_time  = 0;
sample_t ir_local, red_local;

// shared buffer sampling timer isr and loop
volatile sample_t ir_sample, red_sample;

// filter classes
DCFilter red_dc_filter(0.96);
DCFilter ir_dc_filter(0.96);

// variables for display task
const int8_t    max_display_amplitude = 16;
const uint16_t  wave_display_period = 2000/16;
uint32_t last_display_update = 0;
uint8_t wave_frontier_column = 0;


inline void sample(sample_t& sample) {
  int sample_v = analogRead(sample_pin);

  if (inverted)
    sample_v = analog_max - sample_v;

  sample.v  = static_cast<float>(sample_v);
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
  sample.v = dc_filter.step(sample.v);
//  Serial.print("\n in filter v ");
//  Serial.println(sample.v);
  sample.dc = dc_filter.get_dc();
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


void display_task(sample_t& sample) {
  if (sample.t > last_display_update + wave_display_period) {
    last_display_update = sample.t;
    lcd.setCursor((wave_frontier_column+1) & 0xf, 0);
    lcd.write(" ");
    lcd.setCursor((wave_frontier_column+1) & 0xf, 1);
    lcd.write(" ");

    int16_t v_corrected = static_cast<int16_t>(sample.v) + max_display_amplitude;
    if (v_corrected < 0)
      v_corrected = 0;
    if (v_corrected > max_display_amplitude*2-1)
      v_corrected = max_display_amplitude*2-1;
    v_corrected = v_corrected * 16 / (max_display_amplitude * 2);
    uint8_t write_line = v_corrected % 16; 
    if (write_line <= 7)
      lcd.setCursor(wave_frontier_column, 1);
    else
      lcd.setCursor(wave_frontier_column, 0);

    lcd.write(write_line%8);

    
    wave_frontier_column = (wave_frontier_column+1) & 0xf;
  }

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
  Serial.println(ir_local.v);

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

  lcd_set_up();
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

  bool red_update = false;
  bool ir_update = false;

  if (red_local.t != last_red_sample_time) {
    red_update = true;
    last_red_sample_time = red_local.t;
  }

  if (ir_local.t != last_ir_sample_time) {
    ir_update = true;
    last_ir_sample_time = ir_local.t;
  }

  if (ir_update) {
    display_task(ir_local);
  }

  if (red_update || ir_update) {    
    serial_publish_task();
  }
  
}
