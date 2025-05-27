#ifndef BEAT_DETECTOR_H
#define BEAT_DETECTOR_H

#include "types.h"

#define STATE_WAITING  0
#define STATE_UPWARD  1
#define STATE_PENDING_DETECTION  2
#define STATE_MASKED  3
#define SAMPLE_PERIOD  12.5

#define     DROPPING_DEADZONE   5
#define     MIN_THRES           0.2
#define     MAX_THRES           64
#define     V_MAX_CAP           64
#define    UPWARD_SLOP_THRES    0.06
#define     SLOPE_ALPHA_OVER_SAMPLE_PERIOD      0.048    // 0.6/12.5
#define     PERIOD_ALPHA_OVER_SAMPLE_PERIOD     0.048    // 0.6/12.5
#define     NO_BEAT_TIMEOUT     2000
#define      MASK_DELAY         200
#define     THRES_DROP_FACTOR   1.2
#define     THRES_DECAY_FACTOR  0.98

#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif

#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#endif


class beatDetector {
public:
    beatDetector() {
        slope = 0;
        v_last = 0;
        t_last = 0;
        threshold = MIN_THRES;
        state = STATE_WAITING;
        period = 0;
        t_last_peak = 0;
        v_last_peak = -V_MAX_CAP;
    }

    bool step(sample_t& sample) {
        bool peak_detected = false;
        
        uint32_t t = sample.t;
        float v = min(sample.v, V_MAX_CAP);

        float current_slope = (v-v_last)/(float)(t-t_last);
        float slope_alpha_corrected = SLOPE_ALPHA_OVER_SAMPLE_PERIOD*(float)(t-t_last);
        slope = slope_alpha_corrected*current_slope + (1-slope_alpha_corrected)*slope;
        
        bool capped = false;
        if ((v >= V_MAX_CAP-0.5) && (v_last >= V_MAX_CAP-0.5)) {
            state = STATE_WAITING;
            threshold = MAX_THRES;
            capped = true;
        }
            
        if (state == STATE_WAITING) {
            if ((v > threshold) && (slope > UPWARD_SLOP_THRES) && (!capped)) {
                threshold = min(v, MAX_THRES);
                state = STATE_UPWARD;
            }

            if (t - t_last_peak > NO_BEAT_TIMEOUT) {
                v_last_peak = V_MAX_CAP;
                period = 0;
            }
            decrease_threshold(t);
        }
                
        else if (state == STATE_UPWARD) {
            if (v < threshold)
                state = STATE_PENDING_DETECTION;
            else
                threshold = min(max(v, threshold), MAX_THRES);
        }

        else if (state == STATE_PENDING_DETECTION) {
            if (v < threshold - DROPPING_DEADZONE) {
                peak_detected = true;
                state = STATE_MASKED;

                if (v_last_peak > -V_MAX_CAP+0.5) {
                    float delta = (float)(t - t_last_peak);
                    float period_alpha_corrected = PERIOD_ALPHA_OVER_SAMPLE_PERIOD*(float)(t-t_last);
                    period = delta*period_alpha_corrected + (1-period_alpha_corrected)*period;
                    v_last_peak_mul_thres_drop_factor_over_period  = threshold*THRES_DROP_FACTOR/period;
                }
                
                v_last_peak = threshold;
                t_last_peak = t;
            }
            else {
                state = STATE_UPWARD;
            }
        }
        
        else if (state == STATE_MASKED) {
            if (t - t_last_peak > MASK_DELAY)
                state = STATE_WAITING;
            decrease_threshold(t);
        }     

        t_last = t;
        v_last = v;
        return peak_detected;
    }

    uint8_t state;
    float period;
    float threshold;
    uint32_t t_last_peak;
    float   v_last_peak;

    // private:
    float slope;
    float v_last;
    uint32_t t_last;
    float v_last_peak_mul_thres_drop_factor_over_period;

    
    void decrease_threshold(uint32_t t) {
        if ((period > 0) && (v_last_peak > 0))
            threshold -= (v_last_peak_mul_thres_drop_factor_over_period * (float)(t-t_last));
        else
            threshold *= THRES_DECAY_FACTOR;
            
        threshold = max(threshold, MIN_THRES);
    }

};

#endif
