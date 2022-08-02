#include <data_buff.h>
#include <Arduino.h>

Buffer::Buffer(int buf_size){
    buf = (float *)malloc(sizeof(float) * buf_size);
    t_buf = (double *)malloc(sizeof(double) * buf_size);
    n = buf_size;
    clear();
}

void Buffer::insert(double t, float y) {
    // inserts new data point at position pointed by index
    // and recalculates slope

    // recalculate slope
    t_avg += (t - t_buf[curr_i])/double(n);
    y_avg += (y - buf[curr_i])/float(n);
    // t_t += (t*t - t_buf[curr_i]*t_buf[curr_i]);
    // t_y += (t*y - t_buf[curr_i]*buf[curr_i]);
    

    // t_avg = 0; y_avg = 0; t_t = 0; t_y = 0;
    float num = 0;
    float denom = 0;
    for (int i = 0; i<n; i++) {
        num += (t_buf[i]-t_avg) * (buf[i]-y_avg);
        denom += (t_buf[i]-t_avg) * (t_buf[i]-t_avg);
        // t_avg += t_buf[i]/double(n);
        // y_avg += buf[i]/double(n);
        // t_t += t_buf[i] * t_buf[i];
        // t_y += t_buf[i] * buf[i];
    
    }
    // slope = (t_y - n*t_avg*y_avg)/(t_t - n*t_avg*t_avg);
    slope = num/denom;

    // update buffers
    buf[curr_i] = y;
    t_buf[curr_i] = t;

    // increment indices
    curr_i++;
    curr_i = curr_i % n;
    if (curr_i == 0) {
        is_full = true;
    }
}

void Buffer::clear() {
    curr_i = 0;
    is_full = false;
    t_avg = 0;
    y_avg = 0;
    t_t = 0;
    t_y = 0;
    for (int i = 0; i<n; i++) {
        buf[i] = 0;
        t_buf[i] = 0;
    }
}

float Buffer::get_slope() {
    // returns 0 if buffer hasn't been filled
    return is_full ? slope : 0;
}