#include <aubo_kinematics/aubo_kin.h>
#include "ros/ros.h"
#include <math.h>
#include <stdio.h>


namespace aubo_kinematics {

  namespace {

    int SIGN(double x) {
      return (x > 0) - (x < 0);
    }

    float  antiSinCos(float sA,float cA){
        float angle,r;
        r = sqrt(sA*sA+cA*cA);
        sA /= r;
        cA /= r;

        if (fabs(cA) < 1.75e-5)
            angle = M_PI/2*SIGN(sA);
        else
        {
            r = sA/cA;
            angle = atan(r);
            if (cA < 0) angle -= M_PI*SIGN(angle);
        }
        return angle;
    }
  
    #ifdef AUBO_I5_PARAMS

    const double EPS_DEF = 0.0001;
    const double EPS_DEF_J1 = 0.0125;
    const double EPS_DEF_J3 = 0.046;

    const double d1 =  0.0985;
    const double d2 =  0.1215;
    const double a2 =  0.408;
    const double a3 =  0.376;
    const double d4 =  0.0;
    const double d5 =  0.1025;
    const double d6 =  0.094;

    #endif
   
  }

  void forward(const double* q, double* T){
      float s1 = sin(*q + M_PI), c1 = cos(*q + M_PI); q++;
      float s2 = sin(*q - M_PI/2), c2 = cos(*q - M_PI/2); q++;
      float s3 = sin(*q), c3 = cos(*q); q++;
      float s4 = sin(*q - M_PI/2), c4 = cos(*q - M_PI/2); q++;
      float s5 = sin(*q), c5 = cos(*q); q++;
      float s6 = sin(*q), c6 = cos(*q);

      float tmp0 = c2*c3;
      float tmp1 = s2*s3;
      float tmp2 = c1*tmp0 + c1*tmp1;
      float tmp3 = c3*s2;
      float tmp4 = c2*s3;
      float tmp5 = c1*tmp3 - c1*tmp4;
      float tmp6 = c4*tmp2 - s4*tmp5;
      float tmp7 = c5*tmp6 + s1*s5;
      float tmp8 = -c4*tmp5 - s4*tmp2;
      float tmp9 = s5*tmp6;
      float tmp10 = c5*s1;
      float tmp11 = a2*c2;
      float tmp12 = s1*tmp0 + s1*tmp1;
      float tmp13 = s1*tmp3 - s1*tmp4;
      float tmp14 = c4*tmp12 - s4*tmp13;
      float tmp15 = -c1*s5 + c5*tmp14;
      float tmp16 = -c4*tmp13 - s4*tmp12;
      float tmp17 = c1*c5;
      float tmp18 = s5*tmp14;
      float tmp19 = tmp0 + tmp1;
      float tmp20 = -tmp3 + tmp4;
      float tmp21 = -c4*tmp19 - s4*tmp20;
      float tmp22 = c4*tmp20 - s4*tmp19;
      float tmp23 = c5*tmp22;
      float tmp24 = s5*tmp22;

      *T = c6*tmp7 + s6*tmp8;	T++;
      *T = c6*tmp8 - s6*tmp7;	T++;
      *T = -tmp10 + tmp9;	T++;
      *T = a3*tmp2 + c1*tmp11 - d2*s1 - d4*s1 + d5*tmp8 - d6*(tmp10 - tmp9); T++;
      *T = c6*tmp15 + s6*tmp16; T++;
      *T = c6*tmp16 - s6*tmp15;	T++;
      *T = tmp17 + tmp18;	T++;
      *T = a3*tmp12 + c1*d2 + c1*d4 + d5*tmp16 - d6*(-tmp17 - tmp18) + s1*tmp11;	T++;
      *T = c6*tmp23 + s6*tmp21;	T++;
      *T = c6*tmp21 - s6*tmp23;	T++;
      *T = tmp24;	T++;
      *T = -a2*s2 + a3*tmp20 + d1 + d5*tmp21 + d6*tmp24;	T++;
      *T = 0;	T++;
      *T = 0;	T++;
      *T = 0;	T++;
      *T = 1;
  }



  int inverse(const double* T, double *jointResults) {
      float R06[9], positions[3];
      int i, j, j1_idx,j3_idx,j5_idx, num_j1,num_j3,num_j5,solution_num=0;
      float J1[2],J2,J3[2],J4,J5[2],J6,J234,J6234;
      float aCos, bSin,abCos,alpha,SumSJ23,SumCJ23,temp,temp1,temp2;
      float CA_1,SA_1,CA_5,SA_5,CA_3,SA_3,CA_6,SA_6,SA_J234,CA_J234,SA_2,CA_2;
      bool loop_again = false;
      char nof_j5=0;

      for (i=0;i<3;i++)
      {
          for (j=0;j<3;j++)
          {
               R06[i*3 + j] = T[i*4 + j];
          }
          positions[i] = T[i*4 + 3];
      }

      for (i=0;i<3;i++)
      {
          R06[i] = -R06[i];
          R06[3+i] = -R06[3+i];
      }

      positions[0]= -positions[0];
      positions[1]= -positions[1];

      /******************************************************
      * j1
      ******************************************************/
      /* calc alpha first */
      aCos = positions[1]-d6*R06[5];
      bSin = positions[0]-d6*R06[2];
      abCos = sqrt(aCos*aCos+bSin*bSin);
      if (abCos < d2-EPS_DEF_J1)
      {
          return solution_num;
      }
      CA_1 = aCos/abCos;
      SA_1 = bSin/abCos;
      alpha = antiSinCos(SA_1,CA_1);

      //calc j1
      CA_1 = d2/abCos;
      if (CA_1 > 0.9999999998) //0.001 degree
      {
          num_j1 = 1;
          temp = 0;
      }
      else
      {
          num_j1 = 2;
          temp = acos(CA_1);

          J1[1] = alpha-temp;
          if (J1[1] <=-M_PI) J1[1] += 2*M_PI;
      }

      J1[0] = temp+alpha;
      if (J1[0] > M_PI) J1[0] -= 2*M_PI;


      for (j1_idx=0;j1_idx<num_j1;j1_idx++)
      {

          CA_1 = cos(J1[j1_idx]);
          SA_1 = sin(J1[j1_idx]);
          CA_5 = R06[5]*CA_1+R06[2]*SA_1;
          if (!loop_again)
          {
          temp = fabs(CA_5);
          if (temp > 1+EPS_DEF)
          {
              continue;
          }
          else if (temp > 1-6e-8)
          {
              J5[0] = M_PI*(1-SIGN(CA_5))/2;
              num_j5 = 1;
          }
          else
          {
              J5[0] = acos(CA_5);
              J5[1] = -J5[0];
              num_j5 = 2;
          }
          nof_j5 |= (num_j5>1)<<j1_idx;
          }
          else
          {
              if (nof_j5&(1<<j1_idx)) num_j5 = 2;
              else num_j5 = 0;
          }

          for (j5_idx=0;j5_idx<num_j5;j5_idx++)
          {

              if (num_j5 == 1)
              {
                  J6234 = antiSinCos( -R06[6], R06[7]);


                  SumSJ23 = (-positions[0]+d6*R06[2])*CA_1+(positions[1]-d6*R06[5])*SA_1;
                  SumCJ23 = positions[2]-d1-d6*R06[8];
                  temp = fabs(SumSJ23)-(a2+a3+d5);
                  temp1 = fabs(SumCJ23)-(a2+a3+d5);

                  if (fabs(temp)<EPS_DEF || fabs(temp1)<EPS_DEF)
                  {

                      if (fabs(temp)<EPS_DEF) J2 = M_PI/4*SIGN(SumSJ23);
                      else J2 = (1-SIGN(SumCJ23))*M_PI/2;

                      J6 = J6234-J2*SIGN(CA_5);

                      jointResults[solution_num*6] = J1[j1_idx];
                      jointResults[solution_num*6 + 1] = J2;
                      jointResults[solution_num*6 + 2] = 0;
                      jointResults[solution_num*6 + 3] = 0;
                      jointResults[solution_num*6 + 4] = J5[0];
                      jointResults[solution_num*6 + 5] = J6;
                      solution_num++;
                  }
                  else
                  {

                      J2 = 0.0; // maintain J2;
                      //calc alpha
                      temp = SumSJ23*SumSJ23+SumCJ23*SumCJ23;

                      abCos=sqrt(temp);
                      alpha = antiSinCos(SumSJ23/abCos, SumCJ23/abCos);
                      temp1 = (temp+d5*d5-a2*a2-a3*a3)/(2*a2*a3);
                      temp2 = -d5*abCos/(a2*a3); // negative

                      if (fabs(temp1) > 1-temp2+EPS_DEF)
                      {
                          continue;
                      }
                      else if (fabs(temp1) > 1-temp2-EPS_DEF)
                      {

                          J3[0] = (1-SIGN(temp1))*M_PI/2;
                          J234 = J3[0]+alpha; //calc J234
                          while (J234 > M_PI) J234 -= 2*M_PI;
                          while (J234 <= -M_PI) J234 += 2*M_PI;
                      }
                      else
                      {

                          //last j3 in range or not
                          CA_3 = cos(0);
                          temp = temp1+temp2;
                          temp1 -= temp2;
                          if (CA_3 >= temp && CA_3 <= temp1)
                          {
                              J3[0] = 0;
                          }
                          else
                          {
                              if (CA_3 < temp && fabs(temp)<=1) temp2 = temp;
                              else if (CA_3 > temp1 && fabs(temp1)<=1) temp2 = temp1;
                              else
                              {
                                  continue;
                              }
                              J3[0] = acos(temp2);
                              //min change
                              if (fabs(J3[0])>fabs(J3[0])) J3[0] = -J3[0];
                          }
                          //calc J234
                          SA_J234 = (SumSJ23-a2*sin(J2)-a3*sin(J2-J3[0]))/d5;
                          CA_J234 = (SumCJ23-a2*cos(J2)-a3*cos(J2-J3[0]))/d5;
                          J234 = antiSinCos(SA_J234,CA_J234);
                      }
                      J4 = J234-J2+J3[0];
                      J6 = J6234-J234*SIGN(CA_5);
                      jointResults[solution_num*6] = J1[j1_idx];
                      jointResults[solution_num*6 + 1] = J2;
                      jointResults[solution_num*6 + 2] = J3[0];
                      jointResults[solution_num*6 + 3] = J4;
                      jointResults[solution_num*6 + 4] = J5[0];
                      jointResults[solution_num*6 + 5] = J6;
                      solution_num++;
                  }
              }
              else
              {

                  /*-----------------------------------------------------
                  * j6
                  *-----------------------------------------------------*/
                  SA_5 = sin(J5[j5_idx]);
                  CA_6 = (R06[3]*CA_1+R06[0]*SA_1)/SA_5;
                  SA_6 = (R06[4]*CA_1+R06[1]*SA_1)/SA_5;


                  temp = sqrt(CA_6*CA_6+SA_6*SA_6);
                  SA_6 /= temp;
                  CA_6 /= temp;
                  J6 = antiSinCos(SA_6, CA_6);

                  /*-----------------------------------------------------
                  * j234
                  *-----------------------------------------------------*/
                  SA_J234 = R06[8]*SA_5-R06[6]*CA_5*CA_6-R06[7]*CA_5*SA_6;
                  CA_J234 = R06[7]*CA_6-R06[6]*SA_6;
                  J234 = antiSinCos(SA_J234,CA_J234);


                  SumSJ23 = -positions[0]*CA_1 + positions[1]*SA_1 + d6*R06[2]*CA_1 - d6*R06[5]*SA_1 - d5*SA_J234;
                  SumCJ23 =  positions[2] - d1 - d6*R06[8] - d5*CA_J234;
                  CA_3 = (SumSJ23*SumSJ23+SumCJ23*SumCJ23-a2*a2-a3*a3)/(2*a2*a3);

                  if (fabs(CA_3) > 1+(loop_again ? EPS_DEF_J3 : EPS_DEF))
                  {
                      if (loop_again && !solution_num && j5_idx == 1) ;
                      continue;
                  }
                  else if (fabs(CA_3) > 1-5*10e-14)
                  {
                      J3[0] = M_PI*(1-SIGN(CA_3))/2;
                      num_j3 = 1;
                  }
                  else
                  {
                      J3[0] = acos(CA_3);
                      J3[1] = -J3[0];
                      num_j3 = 2;
                  }

                  for (j3_idx=0;j3_idx<num_j3;j3_idx++)
                  {
                      SA_3 = sin(J3[j3_idx]);
                      temp = a2+a3*CA_3;
                      temp1 = a3*SA_3;
                      temp2 = temp*temp+temp1*temp1;
                      CA_2 = (temp*SumCJ23-temp1*SumSJ23)/temp2;
                      SA_2 = (temp*SumSJ23+temp1*SumCJ23)/temp2;
                      if (CA_2*CA_2+SA_2*SA_2 > 1+EPS_DEF)
                      {
                          if (loop_again && !solution_num && j5_idx == 1) ;
                          continue;
                      }
                      J2 = antiSinCos(SA_2,CA_2);
                      J4 = J234-J2+J3[j3_idx];
                      while (J4 <= -M_PI) J4 += 2*M_PI;
                      while (J4 > M_PI) J4 -= 2*M_PI;

                      jointResults[solution_num*6] = J1[j1_idx];
                      jointResults[solution_num*6 + 1] = J2;
                      jointResults[solution_num*6 + 2] = J3[j3_idx];
                      jointResults[solution_num*6 + 3] = J4;
                      jointResults[solution_num*6 + 4] = J5[j5_idx];
                      jointResults[solution_num*6 + 5] = J6;
                      solution_num++;
                  }
              }
          }
          if (j1_idx == num_j1-1 && solution_num == 0 && nof_j5 && !loop_again)
          {
              j1_idx = -1;
              loop_again = true;
          }
      }

      //JOINT_REVERSE
      for (int i=0;i<solution_num*6;i++)
          jointResults[i] = -jointResults[i];
      return solution_num;
  }
};


#define IKFAST_HAS_LIBRARY
#include <aubo_kinematics/ikfast.h>
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==61);

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

void to_mat44(double * mat4_4, const IkReal* eetrans, const IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        mat4_4[i*4+0] = eerot[i*3+0];
        mat4_4[i*4+1] = eerot[i*3+1];
        mat4_4[i*4+2] = eerot[i*3+2];
        mat4_4[i*4+3] = eetrans[i];
    }
    mat4_4[3*4+0] = 0;
    mat4_4[3*4+1] = 0;
    mat4_4[3*4+2] = 0;
    mat4_4[3*4+3] = -1;
}

void from_mat44(const double * mat4_4, IkReal* eetrans, IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        eerot[i*3+0] = mat4_4[i*4+0];
        eerot[i*3+1] = mat4_4[i*4+1];
        eerot[i*3+2] = mat4_4[i*4+2];
        eetrans[i] = mat4_4[i*4+3];
    }
}


IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
  if(!pfree) return false;

  int n = GetNumJoints();
  double q_sols[8*6];
  double T[16];

  to_mat44(T, eetrans, eerot);

  int num_sols = aubo_kinematics::inverse(T,q_sols);

  std::vector<int> vfree(0);

  for (int i=0; i < num_sols; ++i){
    std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
    for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
    solutions.AddSolution(vinfos,vfree);
  }
  return num_sols > 0;
}

IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot)
{
    double T[16];
    aubo_kinematics::forward(j,T);
    from_mat44(T,eetrans,eerot);
}

IKFAST_API int GetNumFreeParameters() { return 1; }
IKFAST_API int* GetFreeParameters() { static int freeparams[] = {5}; return freeparams; }
IKFAST_API int GetNumJoints() { return 6; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif


#ifndef IKFAST_NO_MAIN

using namespace std;
using namespace aubo_kinematics;

int main(int argc, char* argv[])
{
  double q[6] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
  double T[16];
  forward(q, T);
  for(int i=0;i<4;i++) {
    for(int j=i*4;j<(i+1)*4;j++)
      printf("%1.3f ", T[j]);
    printf("\n");
  }
  double q_sols[8*6];
  int num_sols;
  num_sols = inverse(T,q_sols);
  for(int i=0;i<num_sols;i++) 
    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
  for(int i=0;i<=4;i++)
    printf("%f ", M_PI/2.0*i);
  printf("\n");
  return 0;
}
#endif
