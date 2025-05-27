
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
