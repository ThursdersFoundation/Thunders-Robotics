/**
 * @file    gpio.c
 * @brief   Digital I/O init: LEDs, user button (EXTI), motor direction pins,
 *          ultrasonic triggers, lidar motor enable.
 */
#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

static void Gpio_Output(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef init = { 0 };
    init.Pin   = pin;
    init.Mode  = GPIO_MODE_OUTPUT;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(port, &init);
    HAL_GPIO_WritePin(port, (uint16_t)pin, GPIO_PIN_RESET);
}

static void Gpio_Input(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    GPIO_InitTypeDef init = { 0 };
    init.Pin   = pin;
    init.Mode  = GPIO_MODE_INPUT;
    init.Pull  = pull;
    init.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(port, &init);
}

void MX_GPIO_Init(void)
{
    /* Note: on STM32F4 there is no SWJ_CFG remap register; configuring
    ** PB3 (JTAG TDO) as TIM2_CH2 releases the JTAG pin automatically.
    ** SWD (PA13/PA14) stays available for debugging. */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Power-on state: all LEDs off */
    HAL_GPIO_WritePin(GPIOD, LED_STATUS_Pin | LED_ERROR_Pin | LED_COMM_Pin | LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
    Gpio_Output(GPIOD, LED_STATUS_Pin);
    Gpio_Output(GPIOD, LED_ERROR_Pin);
    Gpio_Output(GPIOD, LED_COMM_Pin);
    Gpio_Output(GPIOD, LED_HEARTBEAT_Pin);

    /* Motor direction inputs (push-pull outputs, low = brake on boot) */
    Gpio_Output(GPIOE, MOTOR1_INA_Pin | MOTOR1_INB_Pin |
                       MOTOR2_INA_Pin | MOTOR2_INB_Pin |
                       MOTOR3_INA_Pin | MOTOR3_INB_Pin |
                       MOTOR4_INA_Pin | MOTOR4_INB_Pin);

    /* Ultrasonic triggers */
    Gpio_Output(GPIOB, USONIC_FRONT_TRIG_Pin | USONIC_LEFT_TRIG_Pin);
    Gpio_Output(GPIOC, USONIC_RIGHT_TRIG_Pin | USONIC_REAR_TRIG_Pin);

    /* Lidar motor enable */
    Gpio_Output(GPIOA, LIDAR_MOTO_EN_Pin);

    /* User button: PC13, falling edge -> EXTI15_10 */
    Gpio_Input(GPIOC, USER_BTN_Pin, GPIO_PULLUP);
    GPIO_InitTypeDef btn = { 0 };
    btn.Pin   = USER_BTN_Pin;
    btn.Mode  = GPIO_MODE_IT_FALLING;
    btn.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &btn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
