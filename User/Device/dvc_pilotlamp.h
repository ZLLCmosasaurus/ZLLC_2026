#ifndef DVC_PILOTLAMP
#define DVC_PILOTLAMP

#include "main.h"

enum Enum_PilotLamp_Type
{
    Enum_PilotLamp_Type_Left45Deg = 0,
    Enum_PilotLamp_Type_Right45Deg,
    Enum_PilotLamp_Type_Position1,
    Enum_PilotLamp_Type_Position2,
    Enum_PilotLamp_Type_Position3,
    Enum_PilotLamp_Type_Position4,
    Enum_PilotLamp_Type_TurnOffAll,
};

class Class_PilotLamp
{
public:
    void Init(GPIO_TypeDef *gpio_red, uint16_t gpio_pin_red, GPIO_TypeDef *gpio_green, uint16_t gpio_pin_green, GPIO_TypeDef *gpio_blue, uint16_t gpio_pin_blue);
    void TurnOffAll();
    void TIM_Calculate_PeriodElapsedCallback();
    inline void Set_PilotLamp_Type(Enum_PilotLamp_Type __PilotLamp_Type);
    inline Enum_PilotLamp_Type Get_PilotLamp_Type();
    inline void SetLEDRed(GPIO_PinState state);
    inline void SetLEDBlue(GPIO_PinState state);
    inline void SetLEDGreen(GPIO_PinState state);
    Enum_PilotLamp_Type PilotLamp_Type = Enum_PilotLamp_Type_TurnOffAll;
protected:
    GPIO_TypeDef *LED_gpio_red, *LED_gpio_green, *LED_gpio_blue;
    uint16_t LED_pin_red, LED_pin_green, LED_pin_blue;
    
    uint16_t mod = 0;
};
inline Enum_PilotLamp_Type Class_PilotLamp::Get_PilotLamp_Type()
{
    return (PilotLamp_Type);
}

inline void Class_PilotLamp::SetLEDRed(GPIO_PinState state)
{
    HAL_GPIO_WritePin(LED_gpio_red, LED_pin_red, state);
}

inline void Class_PilotLamp::SetLEDBlue(GPIO_PinState state)
{
    HAL_GPIO_WritePin(LED_gpio_blue, LED_pin_blue, state);
}

inline void Class_PilotLamp::SetLEDGreen(GPIO_PinState state)
{
    HAL_GPIO_WritePin(LED_gpio_green, LED_pin_green, state);
}
#endif // !1