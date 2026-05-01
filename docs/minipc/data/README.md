# 数据通信

## 数据发送

下位机需要向上位机发送陀螺仪解算出的四元数数据，要求`Hamilton`四元数。

正常情况下，直接把解算出的四元数发过去即可，无需做其他处理。

四元数数据在[`dvc_imu.cpp`](../../../User/Device/dvc_imu.cpp)中的`TIM_Calculate_PeriodElapsedCallback`进行解算。

然后被赋值给`INS.q`这个数组中，我们直接使用即可。

首先定义四元数结构体，然后编写对应的解析函数。

```cpp
typedef struct {
    float w, x, y, z;
} Quaternion;

Quaternion Class_IMU::Get_Quaternion(void)
{
    return Quaternion{
        .w = INS.q[0],
        .x = INS.q[1],
        .y = INS.q[2],
        .z = INS.q[3],
    };
}
```

然后在[`dvc_minipc.h`](../../../User/Device/dvc_minipc.h)中的`Transform_Angle_Tx`函数将数据赋值到自身成员变量。

```cpp
Tx_Quaternion = IMU->Get_Quaternion();
```

要发送的数据包的处理在[`dvc_minipc.cpp`](../../../User/Device/dvc_minipc.cpp)中的`Output`函数中。

具体处理需要依据上位机的要求，编辑`Pack_Tx_CAN_A`结构体组成，对数据进行排列。

## 数据接收

数据接收需要根据上位机发送的数据包结构，编辑好`Pack_Rx`结构体，然后在[`dvc_minipc.cpp`](../../../User/Device/dvc_minipc.cpp)中的`Data_Process`函数中做进一步的处理。
