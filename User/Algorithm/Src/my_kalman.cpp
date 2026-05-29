#include "my_kalman.h"

void kalman_Init(my_kalman* kalman ,float __Error_Measure, float __Error_Estimate,float __X, float __P)
{
    kalman->Error_Measure = __Error_Measure;
    kalman->Error_Estimate = __Error_Estimate;
    kalman->X_kalman   = __X;
    kalman->P   = __P; 
}
void Recv_Adjust_PeriodElapsedCallback(my_kalman* kalman)
{
    kalman->X_hat = kalman->X_kalman;                  //最普通的估计方式   A = 1, U = 0;
     kalman->P_hat =  kalman->P +  kalman->Error_Estimate;

    //省略了Now 与 X 测量值的转移，因为H = 1

     kalman->Kalman_Gain =  kalman->P_hat / ( kalman->P_hat +  kalman->Error_Measure);              //因为 H 和 A 等于1退化

     kalman->X_kalman = kalman-> X_kalman +  kalman->Kalman_Gain * ( kalman->Now -  kalman->X_kalman);
     kalman->P = (1.0f -  kalman->Kalman_Gain) *  kalman->P_hat;

     kalman->Out = kalman-> X_kalman;
}
void kalman_set_now(my_kalman* kalman,float __Now)
{
	kalman->Now=__Now;

}