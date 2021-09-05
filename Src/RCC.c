/*
 * RCC.c
 *
 *  Created on: 29 Ağu 2021
 *      Author: win10
 */
#include "RCC.h"


uint32_t SystemCoreClock = 2097152U; /* 32.768 kHz * 2^6 */
uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
uint8_t PLLMulTable[9] = {3U, 4U, 6U, 8U, 12U, 16U, 24U, 32U, 48U};
uint32_t RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}


/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (RCC_GetHCLKFreq() >> APBPrescTable[(RCC->RCC_CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}
uint32_t RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (RCC_GetHCLKFreq()>> APBPrescTable[(RCC->RCC_CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
}
/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t HAL_RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (RCC_GetHCLKFreq()>> APBPrescTable[(RCC->RCC_CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
}


/**
  * @brief  Returns the SYSCLK frequency
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is MSI, function returns a value based on MSI
  *             Value as defined by the MSI range.
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  * @note     If SYSCLK source is HSE, function returns a value based on HSE_VALUE(**)
  * @note     If SYSCLK source is PLL, function returns a value based on HSE_VALUE(**)
  *           or HSI_VALUE(*) multiplied/divided by the PLL factors.
  * @note     (*) HSI_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
  *               16 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) HSE_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
  *                8 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *
  * @note   The result of this function could be not correct when using fractional
  *         value for HSE crystal.
  *
  * @note   This function can be used by the user application to compute the
  *         baud-rate for the communication peripherals or configure other parameters.
  *
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  * @retval SYSCLK frequency
  */
uint32_t RCC_GetSysClockFreq(void)
{
  uint32_t tmpreg, pllm, plld, pllvco, msiclkrange;    /* no init needed */
  uint32_t sysclockfreq;

  tmpreg = RCC->RCC_CFGR;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (tmpreg & RCC_CFGR_SWS)
  {
    case RCC_SYSCLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
    {
      if ((RCC->RCC_CR & RCC_CR_HSIDIVF) != 0U)
      {
        sysclockfreq =  (HSI_VALUE >> 2);
      }
      else
      {
        sysclockfreq =  HSI_VALUE;
      }
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  /* PLL used as system clock */
    {
      pllm = PLLMulTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos];
      plld = ((uint32_t)(tmpreg & RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) + 1U;
      if (RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pllm) / (uint64_t)plld);
      }
      else
      {
        if ((RCC->RCC_CR & RCC_CR_HSIDIVF) != 0U)
        {
          pllvco = (uint32_t)((((uint64_t)(HSI_VALUE >> 2)) * (uint64_t)pllm) / (uint64_t)plld);
        }
        else
        {
         pllvco = (uint32_t)(((uint64_t)HSI_VALUE * (uint64_t)pllm) / (uint64_t)plld);
        }
      }
      sysclockfreq = pllvco;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_MSI:  /* MSI used as system clock source */
    default: /* MSI used as system clock */
    {
      msiclkrange = (RCC->RCC_ICSCR & RCC_ICSCR_MSIRANGE ) >> RCC_ICSCR_MSIRANGE_Pos;
      sysclockfreq = (32768U * (1UL << (msiclkrange + 1U)));
      break;
    }
  }
  return sysclockfreq;
}
