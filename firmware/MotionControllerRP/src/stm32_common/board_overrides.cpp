// Board-level overrides for the STM32F401 production controller.

#include <Arduino.h>
#include <PeripheralPins.h>

// The installed MCU is an STM32F401RET6 (DEV_ID 0x433). With the adapter's
// 25 MHz crystal, run the F401 at its 84 MHz maximum and derive an exact
// 48 MHz clock for USB: 25 / 25 * 336 / 7 = 48 MHz.
#if defined(STM32_CONTROLLER_F401_HSE_25MHZ)
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef oscillator = {};
  RCC_ClkInitTypeDef clocks = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM = 25;
  oscillator.PLL.PLLN = 336;
  oscillator.PLL.PLLP = RCC_PLLP_DIV4;  // 25 / 25 * 336 / 4 = 84 MHz
  oscillator.PLL.PLLQ = 7;              // 25 / 25 * 336 / 7 = 48 MHz USB
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) Error_Handler();

  clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV2;
  clocks.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
#endif

// The generated RET6 USB map also configures PA9 as VBUS, even with VBUS
// sensing disabled. PA9 is the adapter's motor-driver STBY signal, so expose
// only the two pins actually needed for USB device operation.
#if defined(STM32_CONTROLLER_USB_DP_DM_ONLY) && defined(HAL_PCD_MODULE_ENABLED)
extern const PinMap PinMap_USB_OTG_FS[] = {
    {PA_11, USB_OTG_FS,
     STM_PIN_DATA(STM_MODE_AF_PP, LL_GPIO_PULL_UP, GPIO_AF10_OTG_FS)},
    {PA_12, USB_OTG_FS,
     STM_PIN_DATA(STM_MODE_AF_PP, LL_GPIO_PULL_UP, GPIO_AF10_OTG_FS)},
    {NC, NP, 0},
};
#endif
