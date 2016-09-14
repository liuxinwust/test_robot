#ifndef AUBO_KIN_H
#define AUBO_KIN_H


namespace aubo_kinematics {
  // @param q       The 6 joint values 
  // @param T       The 4x4 end effector pose in row-major ordering
  void forward(const double* q, double* T);


  // @param T       The 4x4 end effector pose in row-major ordering
  // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
  // @return        Number of solutions found (maximum of 8)
  int inverse(const double* T,  double* q_sols);
};

#endif //AUBO_KIN_H
