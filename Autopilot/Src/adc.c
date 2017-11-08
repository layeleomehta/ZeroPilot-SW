/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

// All of this code was copied and pasted from the picpilot, verify it is still functionioning here
/*
 * Author: Chris
 *
 * Created on June 15, 2013, 3:40 PM
 */

// Internal analog reference voltage ratio
#define AREF_RATIO (3.3f / 4096)

// Airspeed constants
  // Number of ADC readings to average
  #define AIRSPEED_HISTORY 20
  // Source voltage for the airspeed sensor
  #define VSOURCE 5.0f
  // Airspeed voltage divider ratio
  #define ASPD_RATIO (22f / 33)

// Battery Constants
  #define V_SAMPLE_COUNT 20
  // External battery voltage divider ratio
  #define EXT_BATT_RATIO (22f / 172)
  // Battery voltage divider ratio
  #define MAIN_BATT_RATIO (22f / 172)

// ADC -> voltage -> percentage -> pressure, then multiplied by some factors for Bernoulli
static float v_sc = ((AREF_RATIO / ASPD_RATIO) / VSOURCE) * 0.2f  * 1666.67f; // derived from the internal ADC conversions and the airspeed sensor datasheet
// static float ardu = 1.9936f; // magic number via ardupilot. -- SEEMS TO BE UNUSED?

static int airspeedHistory[AIRSPEED_HISTORY] = {0};
static int historyCounter = 0;

static float offset = 0;

static bool new_data = false;
static float last_speed = 0;

enum ADCPin {
  MAIN_BATT_PIN = 12, // Voltage from main (PICpilot) battery
  EXT_BATT_PIN = 10,  // Voltage from external battery
  SERVO_PIN = 13      // Voltage supply for PWM I/O ports
};

// order for state 
static const uint8_t adc_state[3] = {
  MAIN_BATT_PIN,
  EXT_BATT_PIN,
  //SERVO_PIN
};

static uint8_t cur_state = 0;

static uint32_t current_sum = 0; // current sum of samples
static uint8_t num_samples = 0;

uint16_t main_battery_adc = 0;
uint16_t ext_battery_adc = 0;
uint16_t servo_adc = 0;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO Configuration    
    PB0     ------> ADC2_IN8
    PB1     ------> ADC2_IN9 
    */
    GPIO_InitStruct.Pin = EXT_BATT_ADC_Pin|VBATT_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    /**ADC3 GPIO Configuration    
    PC0     ------> ADC3_IN10 
    */
    GPIO_InitStruct.Pin = AIRSPD_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AIRSPD_ADC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PB0     ------> ADC2_IN8
    PB1     ------> ADC2_IN9 
    */
    HAL_GPIO_DeInit(GPIOB, EXT_BATT_ADC_Pin|VBATT_ADC_Pin);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PC0     ------> ADC3_IN10 
    */
    HAL_GPIO_DeInit(AIRSPD_ADC_GPIO_Port, AIRSPD_ADC_Pin);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*
static void initAirspeedADC(){
  AD1CON1bits.FORM = 0;		// Data Output Format: Unsigned integer
  AD1CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
  AD1CON1bits.ASAM = 1;		// Sampling begins when SAMP bit is set (for now)
  AD1CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
  AD1CON1bits.AD12B = 1;		// 12-bit 2/4-channel operation
  AD1CON1bits.ADDMABM = 0;    // DMA buffers are built in conversion order mode
  AD1CON1bits.SAMP = 1;       // Enable ADC sampling

  AD1CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
  AD1CON2bits.BUFM = 0;
  AD1CON2bits.CHPS = 0;       // Converts channel 0
  AD1CON2bits.VCFG = 0;       // Voltage Reference is 3.3V and Ground Reference is Ground

  AD1CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
  AD1CON3bits.SAMC=0; 		// Auto Sample Time = 0*Tad
  AD1CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

  AD1CHS0bits.CH0SA = 11;     // Channel 0 positive input on AN11 (Sample A)
  AD1CHS0bits.CH0SB = 11;     // Channel 0 positive input on AN11 (Sample B)

  AD1PCFGL = 0;               // Set all pins to analog input
  AD1PCFGH = 0;

  IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
  IEC0bits.AD1IE = 1;			// Enable A/D interrupt
  AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}
  I'm not sure if this is required since the autogenerated setup exists now...
*/

/**
* Converts an ADC signal (0-4095) to an airspeed (m/s).
* Derived from the equation for flow velocity given impact pressure (for fluids)
* @param signal Signal from ADC, in 0-4095
* @return Airspeed, in m/s
*/
static float ADCConvert(uint16_t signal) {
  return sqrtf(signal * v_sc);
}

// run on startup to get a zero value for airspeed. 
// waits until the airspeed buffer is fully populated, then saves the offset 
void calibrateAirspeed() {
  int i;
  int adc_counts = 0;
  for (i = 0; i < AIRSPEED_HISTORY; i++) {
      if (airspeedHistory[i] != 0.0) {
          offset += airspeedHistory[i];
          adc_counts += 1;
      }
  }
  offset /= adc_counts;
  debugInt("Airspeed Offset", (int32_t)offset); // should be pretty close to 2000 (+/- 200)
}

void initAirspeedSensor(){
  //RA11/AN11 is the pin to get the airspeed information
  TRISBbits.TRISB11 = 1;
  initAirspeedADC();
  __delay_ms(200); // delay while the sensor populates the buffer
  calibrateAirspeed();
}

float getCurrentAirspeed(){
  if (new_data) {
      new_data = false;
      int i;
      uint16_t aspd_filtered = 0;
      for (i = 0; i < AIRSPEED_HISTORY; i++){
          aspd_filtered += airspeedHistory[i];
      }
      aspd_filtered /= AIRSPEED_HISTORY;
      last_speed = ADCConvert(abs(aspd_filtered - offset));
  }
  return last_speed;
}

void __attribute__((interrupt, no_auto_psv)) _ADC2Interrupt(void){
  uint16_t input = ADC2BUF0;
  current_sum += input;
  num_samples++;
  if (num_samples >= V_SAMPLE_COUNT) {
      switch (adc_state[cur_state]) {
          case MAIN_BATT_PIN:
              main_battery_adc = current_sum / num_samples;
              break;
          case EXT_BATT_PIN:
              ext_battery_adc = current_sum / num_samples;
              break;
          case SERVO_PIN:
              servo_adc = current_sum / num_samples;
              break;
      }
      
      current_sum = 0;
      num_samples = 0;
      
      cur_state++;
      cur_state %= 3; // increment & loop state counter
      AD2CON1bits.ADON = 0; // disable the ADC
      AD2CHS0bits.CH0SA = adc_state[cur_state]; // change the sample sources
      AD2CHS0bits.CH0SB = adc_state[cur_state];
      AD2CON1bits.ADON = 1; // re-enable the ADC
  }
  IFS1bits.AD2IF = 0;		// Clear the ADC Interrupt Flag
}

/*
static void initBatteryADC() {
  AD2CON1bits.FORM = 0;	 	// Data Output Format: Unsigned integer
  AD2CON1bits.SSRC = 7;		// Internal Counter (SAMC) ends sampling and starts conversion
  AD2CON1bits.ASAM = 1;		// Sampling begins automatically
  AD2CON1bits.SIMSAM = 0;		// Sequential Sampling/conversion
  AD2CON1bits.AD12B = 1;		// 12-bit, 1-channel operation
  AD2CON1bits.ADDMABM = 0; 	// DMA buffers are built in conversion order mode
  AD2CON1bits.SAMP = 1;       // Enable ADC sampling

  AD2CON2bits.SMPI = 0;		// Interrupt address every sample/conversion
  AD2CON2bits.BUFM = 0;
  AD2CON2bits.CHPS = 0;       // Converts channel 0
  AD2CON2bits.VCFG = 0;       // Voltage Reference is 3.3V and Ground Reference is Ground

  AD2CON3bits.ADRC=0;			// ADC Clock is derived from Systems Clock
  AD2CON3bits.SAMC=31; 		// Auto Sample Time = 31*Tad
  AD2CON3bits.ADCS=6;			// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*7 = 175nS

  AD2CHS0bits.CH0SA = 12;     // Channel 0 positive input on AN12 (Sample A)
  AD2CHS0bits.CH0SB = 12;     // Channel 0 positive input on AN12 (Sample B)

  AD2PCFGL = 0;
  
  IFS1bits.AD2IF = 0;			// Clear the A/D interrupt flag bit
  IEC1bits.AD2IE = 1;			// Enable A/D interrupt
  AD2CON1bits.ADON = 1;		// Turn on the A/D converter
}
  Not sure we need this now that the autogenerated code exists
*/ 

void initBatterySensor() {
  //AN10, AN12 and AN13 are the external, main battery, and servo voltage pins, respectively
  TRISBbits.TRISB10 = 1;
  TRISBbits.TRISB12 = 1;
  TRISBbits.TRISB13 = 1;
  initBatteryADC();
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
  new_data = true;
  airspeedHistory[historyCounter++] = ADC1BUF0;
  historyCounter %= AIRSPEED_HISTORY;
  IFS0bits.AD1IF = 0; // Clear the ADC1 Interrupt Flag
}

uint16_t getMainBatteryLevel() {
  // return voltage value * 100
  return (uint16_t)((100.f * (float)main_battery_adc) * (AREF_RATIO / MAIN_BATT_RATIO));
}

uint16_t getExtBatteryLevel() {
  return (uint16_t)((100.f * (float)ext_battery_adc) * (AREF_RATIO / EXT_BATT_RATIO));
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
