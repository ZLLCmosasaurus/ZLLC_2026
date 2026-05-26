#include "dvc_pilotlamp.h"

void Class_PilotLamp::Init(GPIO_TypeDef *gpio_red, uint16_t gpio_pin_red, GPIO_TypeDef *gpio_green, uint16_t gpio_pin_green, GPIO_TypeDef *gpio_blue, uint16_t gpio_pin_blue)
{
    LED_gpio_red = gpio_red;
    LED_pin_red = gpio_pin_red;
    LED_gpio_green = gpio_green;
    LED_pin_green = gpio_pin_green;
    LED_gpio_blue = gpio_blue;
    LED_pin_blue = gpio_pin_blue;
}

void Class_PilotLamp::TurnOffAll()
{
    HAL_GPIO_WritePin(LED_gpio_red, LED_pin_red, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_gpio_green, LED_pin_green, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_gpio_blue, LED_pin_blue, GPIO_PIN_SET);
}

void Class_PilotLamp::TIM_Calculate_PeriodElapsedCallback()
{
    mod = (mod + 1) % 1000;
    switch (PilotLamp_Type)
    {
    case Enum_PilotLamp_Type_Left45Deg:
    {
        SetLEDBlue(GPIO_PIN_SET);
        SetLEDGreen(GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_gpio_red, LED_pin_red, GPIO_PIN_RESET);
    }
    break;
    case Enum_PilotLamp_Type_Right45Deg:
    {
        SetLEDRed(GPIO_PIN_SET);
        SetLEDGreen(GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_gpio_blue,LED_pin_blue,GPIO_PIN_RESET);
    }
    break;
    case Enum_PilotLamp_Type_Position1:
    {
        SetLEDRed(GPIO_PIN_SET);
        SetLEDBlue(GPIO_PIN_SET);
        SetLEDGreen(GPIO_PIN_RESET);
    }
    break;
    case Enum_PilotLamp_Type_Position2:
    {
        SetLEDRed(GPIO_PIN_SET);
        SetLEDBlue(GPIO_PIN_SET);
        if(mod <= 100)
        {
            SetLEDGreen(GPIO_PIN_RESET);
        }
        else SetLEDGreen(GPIO_PIN_SET);
    }
    break;
    case Enum_PilotLamp_Type_Position3:
    {
        SetLEDRed(GPIO_PIN_SET);
        SetLEDBlue(GPIO_PIN_SET);
        if((0 <= mod && mod < 50) || (100 <= mod && mod < 150))
        {
            SetLEDGreen(GPIO_PIN_RESET);
        }
        else SetLEDGreen(GPIO_PIN_SET);
    }
    break;
    case Enum_PilotLamp_Type_Position4:
    {
        SetLEDRed(GPIO_PIN_SET);
        SetLEDBlue(GPIO_PIN_SET);
        if((mod % 100) < 50)
        {
            SetLEDGreen(GPIO_PIN_RESET);
        }
        else SetLEDGreen(GPIO_PIN_SET);
    }
    break;
    case Enum_PilotLamp_Type_TurnOffAll:
    {
        TurnOffAll();
    }
    break;
    default:
        break;
    }
}

void Class_PilotLamp::Set_PilotLamp_Type(Enum_PilotLamp_Type __PilotLamp_Type)
{
    PilotLamp_Type = __PilotLamp_Type;
}


