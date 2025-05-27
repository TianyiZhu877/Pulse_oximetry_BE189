#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>
#define PI 3.14159265358979323846f

// first order Butterworth low pass filter
class butterWorthLPF {
public:
    butterWorthLPF(float fc, float fs) {
        float K  = tanf(PI * fc / fs);
        float inv = 1.0f / (1.0f + K);   

        cf0 = K * inv;        
        cf1 = (1.0f - K) * inv;  
    }

    butterWorthLPF() {  
    // default fc = 5, fs = 80
        cf0 = 0.16591068104035050501;        
        cf1 = 0.66817863791929898998;  
    }
    
    float step(float x) //class II 
    {
        v0 = v1;
        v1 = x * cf0  + cf1 * v0;
        return (v0 + v1);
    }

private:
    float v0, v1, cf0, cf1;
};


class DCFilter {
public:
    DCFilter(float alpha): w(0), alpha(alpha), one_minus_alpha(1-alpha) {
        
    }

    float step(float v) {
        float w_new = v + alpha* w;
        float filtered = w_new-w;
        w = w_new;
        return filtered;
    }

    float get_dc() {
        return one_minus_alpha * w;
    }

private:
    float w;
    float alpha;
    float one_minus_alpha;

};

#endif
