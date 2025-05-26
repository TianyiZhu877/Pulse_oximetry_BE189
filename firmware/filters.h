// #ifndef Q32_MACROS_H
// #define Q32_MACROS_H
#include <stdint.h>

#define NUM_FRAC_BITS 12

// User must define NUM_FRAC_BITS (number of fractional bits, 0 to 31) before including this header
#ifndef NUM_FRAC_BITS
#error "NUM_FRAC_BITS must be defined (0 to 31)"
#endif

#if NUM_FRAC_BITS < 0 || NUM_FRAC_BITS > 31
#error "NUM_FRAC_BITS must be between 0 and 31"
#endif

// Convert a float to Q32 by multiplying by 2^NUM_FRAC_BITS and rounding
#define FLOAT_TO_Q32(f) ((int32_t)((f) * (1LL << NUM_FRAC_BITS) + ((f) >= 0 ? 0.5 : -0.5)))

// Convert a Q32 to float by dividing by 2^NUM_FRAC_BITS
#define Q32_TO_FLOAT(q) ((float)(q) / (1LL << NUM_FRAC_BITS))

// Convert an int16 to Q32 by shifting left by NUM_FRAC_BITS
#define INT16_TO_Q32(i) ((int32_t)(i) << NUM_FRAC_BITS)

// Convert a Q32 to int16 by shifting right by NUM_FRAC_BITS (truncates fractional part)
#define Q32_TO_INT16(q) ((int16_t)((q) >> NUM_FRAC_BITS))

// Multiply two Q32 numbers and adjust to maintain NUM_FRAC_BITS fractional bits
#define Q32_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> NUM_FRAC_BITS))

typedef int32_t   Q32;

// #endif



#define DC_ALPHA        3932    // 0.96 * 2^12
#define ONE_MINUS_DC_ALPHA      164                         



class DCFilter {
public:
    DCFilter(): w(0) {
        
    }

    int16_t step(int16_t v) {
        Q32 w_new = INT16_TO_Q32(v) + Q32_MUL(DC_ALPHA, w);
        int16_t filtered = Q32_TO_INT16(w_new-w);
        w = w_new;
    }

    int16_t get_DC() {
        return Q32_TO_INT16(Q32_MUL(ONE_MINUS_DC_ALPHA, w));
    }

private:
    Q32 w;
    
}
