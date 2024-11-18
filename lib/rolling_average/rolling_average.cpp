  #include <rolling_average.h>
  #include <string.h>

  RollingAVG::RollingAVG(){
    gates_circular_buffer_ele = 0;
    memset(gates_circular_buffer, 0, sizeof(gates_circular_buffer));
    memset(gate_sum_circular_buffer, 0, sizeof(gate_sum_circular_buffer));
    memset(gate_avg_circular_buffer, 0, sizeof(gate_avg_circular_buffer));
  }; 

  void RollingAVG::get_samples(int *arg){
    for (int i = 0; i < num_gates; i++){
      gates_circular_buffer[i][gates_circular_buffer_ele] = *(arg + i + 1);
    }
      //increment to next column element for next loop
    gates_circular_buffer_ele++;
    gates_circular_buffer_ele = gates_circular_buffer_ele%roll_AVG_array_size;
  }

  void RollingAVG::rolling_avg(){

  //Calculate average of circular buffer
  for(int j = 0; j < num_gates; j++){

    gate_sum_circular_buffer[j] = 0;

      for (int i = 0; i < roll_AVG_array_size; i++){
        gate_sum_circular_buffer[j] += gates_circular_buffer[j][i];
      }

    gate_avg_circular_buffer[j] = gate_sum_circular_buffer[j]/roll_AVG_array_size;

  }
  }

  int RollingAVG::avg_value(int i){
    return gate_avg_circular_buffer[i];
  }