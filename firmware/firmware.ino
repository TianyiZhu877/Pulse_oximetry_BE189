  #include <util/atomic.h>
#include "filters.h"
#include "types.h"
// #include "lcd.h"
#include "lcd_frame_buffer.h"
#include "beat_detector.h"
#include <LiquidCrystal_I2C.h>

#define FLOAT_INVALID -0.123456
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
butterWorthLPF red_lpf(6, 80);
butterWorthLPF ir_lpf(6, 80);

// beat detectors
beatDetector ir_beat_detector;
beatDetector red_beat_detector;

//buttons 
const uint16_t debouncingDelay = 300;
uint32_t last_button_time = 0;
const uint8_t ALL_BUTTONS = (1<<PD2) | (1<<PD3);
const uint8_t DISPLAY_MODE_BUTTON = 1<<PD2;
const uint8_t PERIOD_MODE_BUTTON = 1<<PD3;
bool prev_button_pressed = false;

//states
#define WAVE_MODE 0
#define STATS_MODE 1
uint8_t display_mode = STATS_MODE;

// lcd params
int8_t refresh_x_offsets[4] = {0, -2, -4, -6};
lcdFramebuffer frame_buffer(PXIEL_STANDARD_MODE, PXIEL_STANDARD_MODE);
const uint8_t LCD_ADDR = 0x27;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(LCD_ADDR, NROWS, NCOLS);
const uint8_t screen_width = frame_buffer.get_screen_width();
const uint8_t screen_height = 16;

// variables for display task
uint8_t last_write_y = 0;
uint8_t wave_frontier_x = 0;
const int8_t    max_wave_amplitude = 12;
const uint16_t  wave_display_period = 1000/screen_width;
const uint16_t stats_display_period = 20;
uint32_t last_display_update = 0;
uint16_t  last_display_period = -1;
uint16_t  current_bpm = 0xffff;
//spo2
const float spo2_a = -3.3;
const float spo2_b = -21.1;
const float spo2_c = 109.6;
float last_AC1, last_AC2;



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

inline void filter(sample_t& sample, DCFilter& dc_filter, butterWorthLPF& lpf) {
  float v_temp = lpf.step(sample.v);
  sample.v = dc_filter.step(v_temp);
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
      filter(new_sample, red_dc_filter, red_lpf);
      red_sample = new_sample;
    }
    else if (sampling_led == IR) {
       filter(new_sample, ir_dc_filter, ir_lpf);
       ir_sample = new_sample;
    }
  }
  
  isr_count += 1;
  if (isr_count == isr_period) 
    isr_count = 0;
}


void display_task(sample_t& sample, beatDetector& beat_detector) {
  if (display_mode == WAVE_MODE) {

    
    if (sample.t > last_display_update + wave_display_period) {
      last_display_update = sample.t;

      int16_t v_corrected = static_cast<int16_t>(sample.v) + max_wave_amplitude;
      if (v_corrected < 0)
        v_corrected = 0;
      if (v_corrected > max_wave_amplitude*2-1)
        v_corrected = max_wave_amplitude*2-1;
      v_corrected = v_corrected * 16 / (max_wave_amplitude * 2);
      uint8_t write_y = 15 - v_corrected % 16; 

      getCellReturn_t coord;
      frame_buffer.get_cell(coord, wave_frontier_x, last_write_y);
      frame_buffer.set_pixel(coord);
      if (write_y > last_write_y)
        for (uint8_t y = last_write_y+1; y<=write_y; ++y) {
          frame_buffer.get_cell(coord, 0xFF, y);
          frame_buffer.set_pixel(coord);
          // Serial.print(y);
          // Serial.print(" -> ");
          // Serial.println(write_y);
        }

      if (write_y < last_write_y)
        for (uint8_t y = last_write_y-1; y>write_y; --y) {
          frame_buffer.get_cell(coord, 0xFF, y);
          frame_buffer.set_pixel(coord);
          // Serial.print(y);
          // Serial.print(" -> ");
          // Serial.println(write_y);
        }

      last_write_y = write_y;


      // if (coord.pixel_x == 5) {
      //   // display what's on the screen buffer
      //   frame_buffer.refresh_screen(lcd);
      // }

      if (coord.pixel_x == 4) {
        uint8_t next_cell_x = (coord.cell_x+1)%NCOLS;
        frame_buffer.clear_cell_in_pixel_buffer(next_cell_x, 0);
        frame_buffer.clear_cell_in_pixel_buffer(next_cell_x, 1);
      }
      
      if (coord.pixel_x == 0) {
        // would need to refresh the screen
        frame_buffer.reset_character_buffer();
        for (uint8_t i=0; i<4; i++) {
          int8_t cell_x = ((int8_t)(coord.cell_x) + NCOLS + refresh_x_offsets[i])%NCOLS;
          int8_t old_cell_x = ((int8_t)(coord.cell_x) + NCOLS + refresh_x_offsets[i] - 1)%NCOLS;
           frame_buffer.move_idx(lcd, old_cell_x, 0, cell_x, 0, i*2);
           frame_buffer.move_idx(lcd, old_cell_x, 1, cell_x, 1, i*2+1);
        }
      }

      wave_frontier_x = wave_frontier_x+1;
      if (wave_frontier_x > screen_width)
        wave_frontier_x = 0;
    }
  }

  else if (display_mode == STATS_MODE) {
    if (sample.t > last_display_update + stats_display_period ) {
      last_display_update = sample.t;
      
      if (last_display_period != static_cast<uint16_t>(beat_detector.period)) {
        last_display_period = static_cast<uint16_t>(beat_detector.period);
        lcd.setCursor(5, 0);
        if (beat_detector.period > 60) {
          current_bpm = static_cast<uint16_t>(60000.0/beat_detector.period);
          // bpm_valid = true;
          if (current_bpm < 100) lcd.write(' ');
          lcd.print(String(current_bpm));
        } else {
          current_bpm = 0xffff;
          lcd.print("NaN");
          lcd.setCursor(5, 1);
          lcd.print("      ");
        }
      }

      
      lcd.setCursor(4, 0);
      if (beat_detector.state == STATE_UPWARD)
        lcd.write('o');
      else
        lcd.write('@');

    }
  }

}


void spo2_calculate_display_task(float AC1, float DC1, float AC2, float DC2) {
  lcd.setCursor(5, 1);

  if ((last_AC1 != AC1 || last_AC2 != AC2) && (AC1 > 0) && (AC2 > 0) 
      && current_bpm!=0xffff && display_mode == STATS_MODE) {
    
    // Serial.print("calculating spo2: ");
    // Serial.print(AC1);
    // Serial.print(", ");
    // Serial.print(last_AC1);
    // Serial.print(", ");
    // Serial.print(AC2);
    // Serial.print(", ");
    // Serial.println(last_AC2);

    float R = ((last_AC1 + AC1)/2/DC1)/((last_AC2 + AC2)/2/DC2);
    float SpO2 = (spo2_a*R*R + spo2_b*R + spo2_c);
    
    if (SpO2 > 99.9) SpO2 = 99.9;
    if (SpO2 < 75.0) SpO2 = 75.0;
    lcd.print(String(SpO2, 1) + "%  ");
    // else {
    //   lcd.print("NaN    ");
    // }
    
    last_AC1 = AC1;
    last_AC2 = AC2;
  }
}

void serial_publish_task(bool publish_t_dc = false, float extra_value = FLOAT_INVALID) {
  if (extra_value != FLOAT_INVALID) {
    Serial.print(extra_value);
    Serial.print(',');
  }

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

void set_stats_ui_background() {
  lcd.setCursor(0, 0);
  lcd.print("BPM             ");
  lcd.setCursor(0, 1);
  lcd.print("SpO2            ");
}

void button_service_task() {
  uint32_t currTime = millis();
  uint8_t buttons_pind = ~PIND;
  bool current_button = buttons_pind & ALL_BUTTONS;

  if ((!prev_button_pressed) && current_button && (currTime - last_button_time > debouncingDelay)) {
    
    // Serial.print(buttons_pind & ALL_BUTTONS, BIN);
    // Serial.print(", buttons pressed: ");
    // Serial.println(buttons_pind, BIN);
    // if (buttons_pind & HOLD_BUTTON) {
    //   if (hold_state == UPDATING_DISPLAY)  
    //     transit_to_holding();
    //   else if (hold_state == HOLDING) 
    //     transit_to_updating_display();

    // }
    
    if (buttons_pind & DISPLAY_MODE_BUTTON) {
      if (display_mode == WAVE_MODE)  {
        set_stats_ui_background();
        display_mode = STATS_MODE;
      }
      else if (display_mode == STATS_MODE) {
        wave_frontier_x = 0;
        display_mode = WAVE_MODE;
      }

    }

    last_button_time = currTime;
  }

  prev_button_pressed = current_button;
}


void setup() {
  Serial.begin(115200);
  
  float ocr1a_f = CPU_F / prescaler / base_f / 2 - 1;
//  Serial.print("ocr1a: ");
//  Serial.println((uint16_t)ocr1a_f);

  pinMode(red_led_pin, OUTPUT); 
  pinMode(ir_led_pin, OUTPUT); 
  
  DDRD &= ~ALL_BUTTONS;
  PORTD |= ALL_BUTTONS;       // set buttons to input-pullup

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
  
  Wire.setClock(400000);
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  set_stats_ui_background();
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
    ir_beat_detector.step(ir_local);
    display_task(ir_local, ir_beat_detector);
  }

  if (red_update) 
    red_beat_detector.step(red_local);

  if (red_update || ir_update) {    
    spo2_calculate_display_task(red_beat_detector.v_last_peak, red_local.dc, ir_beat_detector.v_last_peak, ir_local.dc);
    serial_publish_task(false, ir_beat_detector.threshold);
  }

  button_service_task();
  
}
