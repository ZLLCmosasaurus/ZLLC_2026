typedef struct
{
		//float Error_Measure = 1.0f;
		float Error_Measure;
    //常量
    float P;                    //协方差矩阵
    float P_hat;                //先验协方差

  //  float H = 1.0f;                    //观测转移矩阵
  //  float A = 1.0f;                    //状态转移矩阵
		float H;
		float A;
    //内部变量

    //估计误差(过程噪声)
  //  float Error_Estimate = 1.0f;
	float Error_Estimate;
    //增益
  //  float Kalman_Gain = 0.0f;
	float Kalman_Gain;
    //读变量

    //输出值
  //  float Out = 0.0f;
	  float Out;

    float X_hat;            //先验估计值
    float X_kalman;                //最终估计值

    //写变量

    //当前值
   // float Now = 0.0f;
		 float Now;
}my_kalman;
void kalman_Init(my_kalman* kalman,float __Error_Measure, float __Error_Estimate,float __X, float __P);
void Recv_Adjust_PeriodElapsedCallback(my_kalman* kalman);
void kalman_set_now(my_kalman* kalman,float __Now);