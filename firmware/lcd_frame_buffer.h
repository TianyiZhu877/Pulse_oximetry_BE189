#include <LiquidCrystal_I2C.h>

#ifndef LCD_FRAME_BUFFER_H
#define LCD_FRAME_BUFFER_H

#define PXIEL_EXTENDED_MODE     0
#define PXIEL_STANDARD_MODE     1


#define NROWS 2
#define NCOLS 16
#define CELL_WIDTH   5
#define CELL_HEIGHT   8
#define CELL_WIDTH_E   6
#define CELL_HEIGHT_E   9

struct getCellReturn_t {
    uint8_t cell_x;
    uint8_t cell_y;
    uint8_t pixel_x;
    uint8_t pixel_y;
};

class lcdFramebuffer {
public:
    lcdFramebuffer(uint8_t x_mode_param = PXIEL_STANDARD_MODE, uint8_t y_mode_param = PXIEL_STANDARD_MODE) {
        y_mode = y_mode_param;
        x_mode = x_mode_param;
        reset_screen_buffer();
        reset_character_buffer();
    }

    void reset_screen_buffer() {
        memset(buffer, 0, sizeof(buffer));
    }
    
    void reset_character_buffer() {
        memset(character_buffer, ' ', sizeof(character_buffer));
    }

    void set_pixel(uint8_t x, uint8_t y) {
        getCellReturn_t coords;
        get_cell(coords, x, y);


        // row major order
        if (coords.pixel_x < CELL_WIDTH && coords.pixel_y < CELL_HEIGHT) {
            uint16_t buffer_coord = coords.cell_x * CELL_HEIGHT * NROWS + coords.cell_y * CELL_HEIGHT + coords.pixel_y;
            buffer[buffer_coord] |= (1<<coords.pixel_x);
        }
    }

    void set_pixel(getCellReturn_t& coords) {
        // row major order
        if (coords.pixel_x < CELL_WIDTH && coords.pixel_y < CELL_HEIGHT) {
            uint16_t buffer_coord = coords.cell_x * CELL_HEIGHT * NROWS + coords.cell_y * CELL_HEIGHT + coords.pixel_y;
            buffer[buffer_coord] |= (1<<coords.pixel_x);
        }
    }

    void unset_pixel(uint8_t x, uint8_t y) {
        getCellReturn_t coords;
        get_cell(coords, x, y);

        // row major order
        if (coords.pixel_x < CELL_WIDTH && coords.pixel_y < CELL_HEIGHT) {
            uint16_t buffer_idx = coords.cell_x * CELL_HEIGHT * NROWS + coords.cell_y * CELL_HEIGHT + coords.pixel_y;
            buffer[buffer_idx] &= ~(1<<coords.pixel_x);
        }
    }
    
    void get_cell(getCellReturn_t&  ret, uint8_t x = 0xff, uint8_t y = 0xff) {
        
        if (x != 0xff) {
            if (x_mode == PXIEL_EXTENDED_MODE) {
                ret.cell_x = x/CELL_WIDTH_E;
                ret.pixel_x = CELL_WIDTH_E - x%CELL_WIDTH_E - 1;
            }
            else {
                ret.cell_x = x/CELL_WIDTH;
                ret.pixel_x = CELL_WIDTH - x%CELL_WIDTH - 1;
            }  
        }

        if (y != 0xff) {
            if (y_mode == PXIEL_EXTENDED_MODE) {
                ret.cell_y = y/CELL_HEIGHT_E;
                ret.pixel_y = y%CELL_HEIGHT_E;
            }
            else {
                ret.cell_y = y/CELL_HEIGHT;
                ret.pixel_y = y%CELL_HEIGHT;
            }
        }

    }

    // will not update character buffer or lcd screen!!
    void clear_cell_in_pixel_buffer(uint8_t cell_x, uint8_t cell_y) {
        uint16_t buffer_idx = cell_x * CELL_HEIGHT * NROWS + cell_y * CELL_HEIGHT;
        for (uint8_t i=0; i<CELL_HEIGHT; i++) {
            buffer[buffer_idx] = 0;
            buffer_idx += 1;
        }
    }

    // void clear_cell

    void refresh_screen(LiquidCrystal_I2C& lcd) {
        for (uint8_t cell_y = 0; cell_y<NROWS; cell_y++) {
            lcd.setCursor(0, cell_y);
            for (uint8_t cell_x = 0; cell_x<NCOLS; cell_x++)
              lcd.write(character_buffer[cell_y][cell_x]);
        }
    }

    void update_cell(LiquidCrystal_I2C& lcd, uint8_t cell_x, uint8_t cell_y, uint8_t idx, bool refresh = false) {
        uint16_t buffer_start_idx = cell_x * CELL_HEIGHT * NROWS + cell_y * CELL_HEIGHT;
        lcd.createChar(idx, buffer+buffer_start_idx);
        character_buffer[cell_y][cell_x] = idx;

        if (refresh) {
            lcd.setCursor(cell_x, cell_y);
            lcd.write(idx);
        }
    }

    void move_idx(LiquidCrystal_I2C& lcd, uint8_t old_cell_x, uint8_t old_cell_y, uint8_t cell_x, uint8_t cell_y, uint8_t idx) {
        lcd.setCursor(old_cell_x, old_cell_y);
        lcd.write(' ');
        character_buffer[old_cell_y][old_cell_x] = ' ';
        update_cell(lcd, cell_x, cell_y, idx, true);
    }



    uint8_t get_screen_width() {
        if (x_mode == PXIEL_EXTENDED_MODE)
            return CELL_WIDTH_E*NCOLS-1;
        return CELL_WIDTH*NCOLS;
    }

    // uint8_t get_cell_width() {
    //     if (x_mode == PXIEL_EXTENDED_MODE)
    //         return CELL_WIDTH_E;
    //     return CELL_WIDTH;
    // }

    uint8_t get_screen_height() {
        if (y_mode == PXIEL_EXTENDED_MODE)
            return CELL_HEIGHT_E*NROWS-1;
        return CELL_HEIGHT*NROWS;
    }


    // uint8_t get_cell_height() {
    //     if (x_mode == PXIEL_EXTENDED_MODE)
    //         return CELL_WIDTH_E;
    //     return CELL_WIDTH;
    // }


    byte buffer[NROWS*NCOLS*CELL_HEIGHT];
    char character_buffer[NROWS][NCOLS];
    uint8_t x_mode;
    uint8_t y_mode;
};


#endif
