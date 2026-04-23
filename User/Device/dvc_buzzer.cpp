#include "dvc_buzzer.h"

// 定义不同优先级下的蜂鸣器音调列表的音调数量
#define BUZZER_FORCE_STOP_PRIORITY_NUM 10
#define BUZZER_DEVICE_OFFLINE_PRIORITY_NUM 10
#define BUZZER_CALIBRATING_PRIORITY_NUM 5
#define BUZZER_CALIBRATED_PRIORITY_NUM 10
#define BUZZER_DJI_STARTUP_PRIORITY_NUM 3
#define BUZZER_FREE_PRIORITY_NUM 10

// 定义强制停止优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_FORCE_STOP_PRIORITY_TONELIST[2][BUZZER_FORCE_STOP_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
// 定义设备离线优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[2][BUZZER_DEVICE_OFFLINE_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
// 定义校准中优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_CALIBRATING_PRIORITY_TONELIST[2][BUZZER_CALIBRATING_PRIORITY_NUM] = {{TONE_A3, TONE_A4, TONE_A3,TONE_D4, TONE_C5},
                                                                                   {1000,2000, 3000, 4000, 8000}};
// 定义已校准优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_CALIBRATED_PRIORITY_TONELIST[2][BUZZER_CALIBRATED_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
// 定义启动优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_DJI_STARTUP_PRIORITY_TONELIST[2][BUZZER_DJI_STARTUP_PRIORITY_NUM] = {{TONE_A3, TONE_C4,TONE_C5},
                                                                                    {3000, 6000, 9000}};
// 定义自由优先级下的蜂鸣器音调列表，两个数组分别表示音调和持续时间
int BUZZER_FREE_PRIORITY_TONELIST[2][BUZZER_FREE_PRIORITY_NUM] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                                                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};


// 定义一个类 Class_Sheet_Music，用于管理乐谱
void Class_Sheet_Music::SetNum(uint8_t __Num){
    // 设置乐谱中的音符数量
    Num = __Num;
}

uint8_t Class_Sheet_Music::GetNum(void){
    // 获取乐谱中的音符数量
    return Num;
}

void Class_Sheet_Music::SetToneList(int *__Tone, int *__Duration, uint8_t __Num){
    // 设置乐谱中的音符列表
    for (size_t i = 0; i < __Num; i++)
    {
        // 设置每个音符的音调和持续时间
        ToneList[i].SetTone((BUZZER_BSP_TONE_TO_POW_LUT_H)__Tone[i]);
        ToneList[i].SetDuration(__Duration[i]);
    }
    // 更新乐谱中的音符数量
    SetNum(__Num);
  }

// 定义一个类 Class_Buzzer_Tone，用于管理单个音符
void Class_Buzzer_Tone::SetTone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone){
    // 设置音符的音调
      Tone  =   __Tone;
  }

void Class_Buzzer_Tone::SetDuration(float __Duration){
    // 设置音符的持续时间
      Duration  =   __Duration;
  }

  BUZZER_BSP_TONE_TO_POW_LUT_H Class_Buzzer_Tone::GetTone(void){
    // 获取音符的音调
      return Tone;
  }

float Class_Buzzer_Tone::GetDuration(void){
    // 获取音符的持续时间
      return Duration;
  }

// 定义一个类 Class_Buzzer，用于管理蜂鸣器
void Class_Buzzer::Buzzer_SheetListInit(void){
    // 初始化乐谱列表
    memset(SheetMusic, 0, sizeof(SheetMusic));
    // 设置不同优先级的乐谱
    SheetMusic[BUZZER_FORCE_STOP_PRIORITY].SetToneList(BUZZER_FORCE_STOP_PRIORITY_TONELIST[0], BUZZER_FORCE_STOP_PRIORITY_TONELIST[1], BUZZER_FORCE_STOP_PRIORITY_NUM);
    SheetMusic[BUZZER_DEVICE_OFFLINE_PRIORITY].SetToneList(BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[0], BUZZER_DEVICE_OFFLINE_PRIORITY_TONELIST[1], BUZZER_DEVICE_OFFLINE_PRIORITY_NUM);
    SheetMusic[BUZZER_CALIBRATING_PRIORITY].SetToneList(BUZZER_CALIBRATING_PRIORITY_TONELIST[0], BUZZER_CALIBRATING_PRIORITY_TONELIST[1], BUZZER_CALIBRATING_PRIORITY_NUM);
    SheetMusic[BUZZER_CALIBRATED_PRIORITY].SetToneList(BUZZER_CALIBRATED_PRIORITY_TONELIST[0], BUZZER_CALIBRATED_PRIORITY_TONELIST[1], BUZZER_CALIBRATED_PRIORITY_NUM);
    SheetMusic[BUZZER_DJI_STARTUP_PRIORITY].SetToneList(&BUZZER_DJI_STARTUP_PRIORITY_TONELIST[0][0], BUZZER_DJI_STARTUP_PRIORITY_TONELIST[1], BUZZER_DJI_STARTUP_PRIORITY_NUM);
    SheetMusic[BUZZER_FREE_PRIORITY].SetToneList(BUZZER_FREE_PRIORITY_TONELIST[0], BUZZER_FREE_PRIORITY_TONELIST[1], BUZZER_FREE_PRIORITY_NUM);
}

void Class_Buzzer::Buzzer_Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel,uint32_t __Driver_Frequency) {
    // 初始化蜂鸣器
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel = __Driver_PWM_TIM_Channel;
    ClkFreq = __Driver_Frequency;
    Psc =   Driver_PWM_TIM->Instance->PSC;
    Arr =   Driver_PWM_TIM->Instance->ARR;
    Buzzer_SheetListInit();
    NowTask= BUZZER_FREE_PRIORITY;
}

void Class_Buzzer::Buzzer_On(void) {
    // 打开蜂鸣器
    HAL_TIM_PWM_Start(Driver_PWM_TIM, Driver_PWM_TIM_Channel);
}

void Class_Buzzer::Buzzer_Off(void) {
    // 关闭蜂鸣器
    HAL_TIM_PWM_Stop(Driver_PWM_TIM, Driver_PWM_TIM_Channel);
}

void Class_Buzzer::Set_State(BUZZER_SWITCH_STATE_E __State) {
    // 设置蜂鸣器的状态
    BuzzerState   =   __State;
    if (__State == BUZZER_SWITCH_ON) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }
}

void Class_Buzzer::Set_Arr(uint16_t __Arr) {
    // 设置蜂鸣器的ARR值
    Arr = __Arr;
    Driver_PWM_TIM->Instance->ARR = Arr;
    __HAL_TIM_SetCompare(Driver_PWM_TIM, Driver_PWM_TIM_Channel, Arr/3*2);
}

void Class_Buzzer::Set_NowTime(float __NowTime) {
    // 设置当前时间
    BuzzerNowTime = __NowTime;
}

void Class_Buzzer::Set_StartTime(float __StartTime) {
    // 设置开始时间
    BuzzerStartTime = __StartTime;
}

void Class_Buzzer::Set_TargetFrequency(float __TargetFrequency) {
    // 设置目标频率
    TargetFrequency = __TargetFrequency;
}

void Class_Buzzer::Play_Tone(BUZZER_BSP_TONE_TO_POW_LUT_H __Tone){
    // 播放指定音调的音符
    Set_TargetFrequency(TONE_A4_FREQUENCY*pow(SEMITONE_COEFFICIENT, __Tone));
    Set_Arr((uint16_t)(ClkFreq / Psc / TargetFrequency));
}

void Class_Buzzer::Reload_Buzzer_Status_PeriodElapsedCallback(void) {
    // 定时器周期结束时重新加载蜂鸣器状态
    if(Get_NowTask()!=Get_LastTask()&&Get_NowTask()!=BUZZER_FREE_PRIORITY) {
        Set_State(BUZZER_SWITCH_ON);
        Set_StartTime(HAL_GetTick());
    }
}

void Class_Buzzer::Set_NowTask(BUZZER_BEEP_TASK_PRIORITY_E __NowTask) { 
    // 设置当前任务
        if(__NowTask<=NowTask) {
        NowTask=__NowTask;
        }
  }
void Class_Buzzer::Set_LastTask(BUZZER_BEEP_TASK_PRIORITY_E __LastTask){
    // 设置上一个任务
    LastTask = __LastTask;
  }

BUZZER_BEEP_TASK_PRIORITY_E Class_Buzzer::Get_NowTask(void){
    // 获取当前任务
    return NowTask;
  }
BUZZER_BEEP_TASK_PRIORITY_E Class_Buzzer::Get_LastTask(void){
    // 获取上一个任务
    return LastTask;
  }

void Class_Buzzer::Set_WaitTime(void) {
    // 设置等待时间
    BuzzerWaitTime = BuzzerNowTime - BuzzerStartTime;
  }
float Class_Buzzer::Get_WaitTime(void) {
    // 获取等待时间
    return BuzzerWaitTime;
  }

uint8_t i = 0;
void Class_Buzzer::PlayToneList(BUZZER_BEEP_TASK_PRIORITY_E __NowTask){
    // 播放当前任务的音符列表
    Set_NowTime(HAL_GetTick());
    Set_WaitTime();
    if((Get_WaitTime()<SheetMusic[__NowTask].ToneList[i].GetDuration()+BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE)   && i    <   SheetMusic[__NowTask].GetNum()) {
        Play_Tone(SheetMusic[__NowTask].ToneList[i].GetTone());
    }
    else if(Get_WaitTime()>=SheetMusic[__NowTask].ToneList[i].GetDuration()+BUZZER_TASK_TICK_DIFFERENCE_TOLERANCE  && i < SheetMusic[__NowTask].GetNum()){
        i++;
    }
    else{
        Set_State(BUZZER_SWITCH_OFF);
        NowTask= BUZZER_FREE_PRIORITY;
				i=0;
    }
    
}

void Class_Buzzer::Buzzer_Calculate_PeriodElapsedCallback(void) {
    // 定时器周期结束时计算蜂鸣器状态
  Reload_Buzzer_Status_PeriodElapsedCallback();
  switch(Get_NowTask()) {
    case BUZZER_FREE_PRIORITY:
        Set_State(BUZZER_SWITCH_OFF);
      break;
    case BUZZER_FORCE_STOP_PRIORITY:
        PlayToneList(BUZZER_FORCE_STOP_PRIORITY);
      break;
    case BUZZER_DEVICE_OFFLINE_PRIORITY:
        PlayToneList(BUZZER_DEVICE_OFFLINE_PRIORITY);
      break;
    case BUZZER_CALIBRATING_PRIORITY:
        PlayToneList(BUZZER_CALIBRATING_PRIORITY);
      break;
    case BUZZER_CALIBRATED_PRIORITY:
        PlayToneList(BUZZER_CALIBRATED_PRIORITY);
      break;
    case BUZZER_DJI_STARTUP_PRIORITY:
        PlayToneList(BUZZER_DJI_STARTUP_PRIORITY);
      break;
   
  }
  Set_LastTask(Get_NowTask());
}