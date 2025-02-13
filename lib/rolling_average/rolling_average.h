#pragma once
#ifndef ROLLING_AVERAGE_H

#define roll_AVG_array_size 10
#define num_gates 8

class RollingAVG{
    public:
        RollingAVG();
        void get_samples(int *arg);
        void get_sample_set(int *arg);
        void rolling_avg();
        int avg_value(int i);
    private:

        int gates_circular_buffer[num_gates][roll_AVG_array_size];
        int gates_circular_buffer_ele; //keeps track of which element for the circular buffer gate

        int gate_sum_circular_buffer[num_gates];
        int gate_avg_circular_buffer[num_gates];
};

#endif //end if for rolling average 