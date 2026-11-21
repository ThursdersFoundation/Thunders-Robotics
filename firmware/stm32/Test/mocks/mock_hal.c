/** @file mock_hal.c : see mock_hal.h
 *
 * Host mock strategy: real driver sources are linked against this mock.
 * Fake peripheral handles point at scratch register blocks, so the real
 * register-level code paths execute; GPIO writes and UART TX are recorded.
 */
#include "mock_hal.h"
#include "stm32f4xx_hal.h"
#include "watchdog_manager.h"
#include "string.h"

/* Scratch peripheral register blocks */
static TIM_TypeDef s_fakeTim2, s_fakeTim3, s_fakeTim4, s_fakeTim1, s_fakeTim8, s_fakeTim5, s_fakeTim6;
static USART_TypeDef s_fakeUart1, s_fakeUart2, s_fakeUart3, s_fakeUart4;
static I2C_TypeDef s_fakeI2c1;
static SPI_TypeDef s_fakeSpi2;
static ADC_TypeDef s_fakeAdc1;
static CAN_TypeDef s_fakeCan1;

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim6, htim8;
UART_HandleTypeDef huart1, huart2, huart3, huart4;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;

/* Bind fake registers before any test runs */
__attribute__((constructor)) static void Mock_BindHandles(void)
{
    htim1.Instance = &s_fakeTim1;   htim2.Instance = &s_fakeTim2;
    htim3.Instance = &s_fakeTim3;   htim4.Instance = &s_fakeTim4;
    htim5.Instance = &s_fakeTim5;   htim6.Instance = &s_fakeTim6;
    htim8.Instance = &s_fakeTim8;
    huart1.Instance = &s_fakeUart1; huart2.Instance = &s_fakeUart2;
    huart3.Instance = &s_fakeUart3; huart4.Instance = &s_fakeUart4;
    hi2c1.Instance = &s_fakeI2c1;
    hspi2.Instance = &s_fakeSpi2;
    hadc1.Instance = &s_fakeAdc1;
    hcan1.Instance = &s_fakeCan1;
}

static volatile uint32_t s_tick = 0U;
void Mock_SetTick(uint32_t ms)     { s_tick = ms; }
void Mock_AdvanceTick(uint32_t ms) { s_tick += ms; }

uint32_t HAL_GetTick(void) { return s_tick; }
void HAL_IncTick(void)     { s_tick++; }
void HAL_Delay(uint32_t ms){ s_tick += ms; }
HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
void HAL_MspInit(void) {}

/** Script a timer counter (used to emulate quadrature pulses). */
void Mock_SetTimerCount(TIM_HandleTypeDef *htim, uint16_t cnt) { htim->Instance->CNT = cnt; }
uint16_t Mock_GetTimerCount(TIM_HandleTypeDef *htim) { return (uint16_t)htim->Instance->CNT; }

/* GPIO recording ------------------------------------------------------ */
static MockGpioStat s_gpio[80];
void Mock_GpioReset(void) { memset(s_gpio, 0, sizeof(s_gpio)); }

static MockGpioStat *GpioFind(uint32_t portId, uint32_t pin)
{
    uint32_t bit = 0U, p = pin;
    while (p >>= 1) { bit++; }
    uint32_t idx = portId * 16U + bit;
    if (s_gpio[idx].writes == 0U) { s_gpio[idx].portId = portId; s_gpio[idx].pin = pin; }
    return &s_gpio[idx];
}

MockGpioStat Mock_GpioStat(uint32_t portId, uint32_t pin) { return *GpioFind(portId, pin); }

static uint32_t PortId(GPIO_TypeDef *port)
{
    if (port == GPIOA) return 0U;
    if (port == GPIOB) return 1U;
    if (port == GPIOC) return 2U;
    if (port == GPIOD) return 3U;
    return 4U;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    MockGpioStat *s = GpioFind(PortId(port), pin);
    s->writes++;
    s->lastState = state;
}

uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return 0U;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    MockGpioStat *s = GpioFind(PortId(port), pin);
    s->writes++;
    s->lastState ^= 1U;
}

/* UART TX capture ------------------------------------------------------ */
uint8_t mock_uart_tx[4][256];
volatile uint16_t mock_uart_tx_len[4];
void Mock_UartReset(void) { memset(mock_uart_tx, 0, sizeof(mock_uart_tx)); memset((void *)mock_uart_tx_len, 0, sizeof(mock_uart_tx_len)); }

static int UartIdx(USART_TypeDef *inst)
{
    if (inst == USART1) return 0;
    if (inst == USART2) return 1;
    if (inst == USART3) return 2;
    if (inst == UART4)  return 3;
    return 0;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)Timeout;
    int idx = UartIdx(huart->Instance);
    if (Size > 255U) { Size = 255U; }
    for (uint16_t i = 0U; i < Size; i++) { mock_uart_tx[idx][i] = pData[i]; }
    mock_uart_tx_len[idx] = Size;
    return HAL_OK;
}

/* Watchdog mock (real watchdog_manager.c is NOT linked on host) --------- */
static int s_watchdogRefreshes;
bool Watchdog_Start(uint32_t timeout_ms) { (void)timeout_ms; return true; }
void Watchdog_Refresh(void) { s_watchdogRefreshes++; }
bool Watchdog_WasResetByWatchdog(void) { return false; }
bool Watchdog_IsStarted(void) { return true; }
int Mock_WatchdogRefreshes(void) { return s_watchdogRefreshes; }

/* I2C mock: MPU6050 register-file emulation (WHO_AM_I + raw burst data) -- */
static uint8_t s_mpuRegs[0x80];
void Mock_I2cSetWhoAmI(uint8_t id) { s_mpuRegs[0x75] = id; }
void Mock_I2cSetBurst14(const uint8_t *raw) { memcpy(&s_mpuRegs[0x3B], raw, 14U); }

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t reg,
                                    uint16_t size, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)hi2c; (void)addr; (void)size; (void)timeout;
    if (reg + len < sizeof(s_mpuRegs)) { memcpy(&s_mpuRegs[reg], data, len); }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t reg,
                                   uint16_t size, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)hi2c; (void)addr; (void)size; (void)timeout;
    if (reg + len < sizeof(s_mpuRegs))
    {
        memcpy(data, &s_mpuRegs[reg], len);
        return HAL_OK;
    }
    return HAL_ERROR;
}
