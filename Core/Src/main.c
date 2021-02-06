/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

///#include <bme280.h>
#include <rfm95.h>
#include <payload.h>
///#include <lorawan.h>
///#include <secconfig.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct pins {
  GPIO_TypeDef *port;
  uint32_t     pin;
} pin_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// no LED and no UART after this frame
#define QUIET_FRAME 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// BME280 driver requires SPI read/write interface functions to use only one uint8 as device id.
// STM32 pins in LL libs are defined by port and pin so pin alone cannot be used as device id.
// To be able to reuse the same SPI functions for RFM and BME280, I use a dev -> port/pin mapping table
pin_t pins[] = {
///  { BME_CS_GPIO_Port, BME_CS_Pin },
  { RFM_CS_GPIO_Port, RFM_CS_Pin },
  { RFM_D0_GPIO_Port, RFM_D0_Pin },
  { RFM_D5_GPIO_Port, RFM_D5_Pin }
};

// must correspond to array index above
typedef enum {
///  BME280_CS_PIN_ID,
  RFM95_NSS_PIN_ID,
  RFM95_DIO0_PIN_ID,
  RFM95_DIO5_PIN_ID
} pin_id_t;

///struct bme280_dev bme280_dev;
///struct bme280_data bme280_data;
///uint32_t bme280_delay;

rfm95_t rfm95_dev;
uint8_t rfm95_ver;
///lorawan_t lorawan;
uint16_t frame_counter;
payload_t payload = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void putstr( const char *str ) {
  if( frame_counter > QUIET_FRAME )  return;
  LL_LPUART_Enable(LPUART1);
  while (*str) {
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1));
    LL_LPUART_TransmitData8(LPUART1, (uint8_t)*str++);
  }
  while (!LL_LPUART_IsActiveFlag_TC(LPUART1));
  LL_LPUART_Disable(LPUART1);
}


void putul( unsigned long u ) {
  if( frame_counter > QUIET_FRAME )  return;
  char num[11];
  char *d = &num[sizeof(num)-1];
  *d = '\0';
  if( u ) {
    while(u) {
      *(--d) = '0' + u % 10;
      u /= 10;
    }
  }
  else {
    *(--d) = '0';
  }
  putstr(d);
}


void putl( long u ) {
  if( u < 0 ) {
    putstr("-");
    u = -u;
  }
  putul(u);
}


void puthex( uint8_t val ) {
  if( frame_counter > QUIET_FRAME )  return;
  static char hex[] = "0123456789abcdef";
  char msg[3];
  msg[0] = hex[val >> 4];
  msg[1] = hex[val & 0xf];
  msg[2] = '\0';
  putstr(msg);
}


void putlhex( uint32_t val ) {
	puthex((val >> 24) & 0xff);
	puthex((val >> 16) & 0xff);
	puthex((val >> 8) & 0xff);
	puthex(val & 0xff);
}


int8_t duplexSpi(uint8_t dev, uint8_t addr, uint8_t *data, uint16_t len) {
  LL_SPI_Enable(SPI1);
  LL_GPIO_ResetOutputPin(pins[dev].port, pins[dev].pin);

  LL_SPI_TransmitData8(SPI1, addr);

  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));   // wait for rx to finish
  LL_SPI_ReceiveData8(SPI1);                 // avoid overflow

  while (len--) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));  // wait for tx to finish
    LL_SPI_TransmitData8(SPI1, *data);       // send data or generate clock for slave

    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)); // wait for rx to finish
    *data = LL_SPI_ReceiveData8(SPI1);       // read receive result
    data++;
  }

  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  LL_GPIO_SetOutputPin(pins[dev].port, pins[dev].pin);
  LL_SPI_Disable(SPI1);

  return 0;
}


uint8_t readPin(uint8_t dev) {
  return LL_GPIO_IsInputPinSet(pins[dev].port, pins[dev].pin);
}


uint8_t *serialize(uint8_t *data, uint32_t value, size_t size) {
  while( size-- ) {
    *(data++) = value & 0xff;
    value >>= 8;
  }
  return data;
}


// use my measured values at ambient temperature for calibration if CAL1 is not available (STM32L011)
#ifndef TEMPSENSOR_CAL1_TEMP
#define TEMPSENSOR_CAL1_TEMP 20
#endif
#ifndef TEMPSENSOR_CAL1_ADDR
#define TEMPSENSOR_CAL1_RAW 1022
#else
#define TEMPSENSOR_CAL1_RAW (*TEMPSENSOR_CAL1_ADDR)
#endif

int32_t deci_celsius( uint32_t raw ) {
  int32_t dc = (int32_t)raw - TEMPSENSOR_CAL1_RAW;
  dc *= (int32_t)TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP;
  dc *= 10;
  dc /= (int32_t)*TEMPSENSOR_CAL2_ADDR - TEMPSENSOR_CAL1_RAW;
  dc += (int32_t)TEMPSENSOR_CAL1_TEMP * 10;
  return dc;
}

uint32_t getAdc( uint32_t *mvbat, uint32_t *mvaccu, uint32_t *mvcc, int32_t *dcelsius ) {
  LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  LL_ADC_Enable(ADC1);
  while( !LL_ADC_IsActiveFlag_ADRDY(ADC1) );


  LL_ADC_REG_StartConversion(ADC1);
  while( !LL_ADC_IsActiveFlag_EOC(ADC1) );
  *mvbat = LL_ADC_REG_ReadConversionData12(ADC1);

  LL_ADC_REG_StartConversion(ADC1);
  while( !LL_ADC_IsActiveFlag_EOC(ADC1) );
  *mvaccu = LL_ADC_REG_ReadConversionData12(ADC1);

  LL_ADC_REG_StartConversion(ADC1);
  while( !LL_ADC_IsActiveFlag_EOC(ADC1) );
  *mvcc = LL_ADC_REG_ReadConversionData12(ADC1);
  *mvcc = VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR) / *mvcc;
  *mvbat = *mvcc * *mvbat / 4095;
  *mvaccu = *mvcc * *mvaccu / 4095;

  LL_ADC_REG_StartConversion(ADC1);
  while( !LL_ADC_IsActiveFlag_EOC(ADC1) );
  uint32_t raw = LL_ADC_REG_ReadConversionData12(ADC1);
  *dcelsius = deci_celsius(raw);

  while( !LL_ADC_IsActiveFlag_EOS(ADC1) );
  LL_ADC_Disable(ADC1);

  return raw;
}

void rfm_setup( uint32_t seed ) {
  rfm95_dev.nss_pin_id = RFM95_NSS_PIN_ID;
  rfm95_dev.dio0_pin_id = RFM95_DIO0_PIN_ID;
  rfm95_dev.dio5_pin_id = RFM95_DIO5_PIN_ID;
  rfm95_dev.spi_write = duplexSpi;
  rfm95_dev.spi_read = duplexSpi;
  rfm95_dev.delay = LL_mDelay;
  rfm95_dev.pin_read = readPin;

  //while( rfm95_ver != 0x12 ) {
    rfm95_ver = rfm95_init(&rfm95_dev, seed);
  //}

  ///lorawan_init(&lorawan, &rfm95_dev);
  ///lorawan_set_keys(&lorawan, NwkSkey, AppSkey, DevAddr);

  // Print test package
  if( 0 ) {
    uint8_t buf[8];
    for( size_t i=0; i<sizeof(buf); i++ ) {
      buf[i] = i;
    }

    ///lorawan_send_data(&lorawan, buf, sizeof(buf), 0);
    unsigned char RFM_Data[] = "hello test world!";
    unsigned char RFM_Package_Length = sizeof(RFM_Data);
    rfm95_send(&rfm95_dev, RFM_Data, RFM_Package_Length);
  }
}


void lora_tx() {
    putstr(" standbys:"); putul(frame_counter);

    ///if( bme_read() ) {
    ///  bme_print();

    uint32_t mvbat = 0, mvaccu = 0, mvcc = 0;
    int32_t dcelsius = 0;

    uint32_t raw_temp = getAdc( &mvbat, &mvaccu, &mvcc, &dcelsius );
    payload.mVcc = mvcc;
    payload.mVbat = mvbat;
    payload.mVaccu = mvaccu;
    payload.dCelsius = (int16_t)dcelsius;

    putstr(" mVcc:"); putul(payload.mVcc);
    putstr(" mVbat:"); putul(payload.mVbat);
    putstr(" mVaccu:"); putul(payload.mVaccu);
    putstr(" dCelsius:"); putl(payload.dCelsius);
    putstr(" rawTemp:"); putul(raw_temp);

    // uint8_t Data[/* sizeof(bme280_data) + */ sizeof(frame_counter) + sizeof(mV)];
    // uint8_t *data = Data;
    // data = serialize(data, mV, sizeof(mV));
    // data = serialize(data, frame_counter, sizeof(frame_counter));
    ///data = serialize(data, (uint32_t)bme280_data.temperature, sizeof(bme280_data.temperature));
    ///data = serialize(data, bme280_data.humidity, sizeof(bme280_data.humidity));
    ///serialize(data, bme280_data.pressure, sizeof(bme280_data.pressure));

    putstr(" send ");
    payload_make_valid(&payload);
    putul(rfm95_send(&rfm95_dev, (uint8_t *)&payload, sizeof(payload))/1000);
    ///putul(lorawan_send_data(&lorawan, Data, sizeof(Data), frame_counter)/1000);
    putstr(" kHz");
    ///}

    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

    if( frame_counter == QUIET_FRAME ) {
      putstr(" continue quietly\n");
    }

    if( frame_counter >= QUIET_FRAME ) {
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, INTERVAL_S * 5, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, ++frame_counter);

    // INTERVAL_S defined in *.ioc, Pinout & Configuration, RTC, Configuration
    putstr(" wake:"); putul(INTERVAL_S); putstr(" s");

    HAL_PWREx_EnableUltraLowPower();
    HAL_PWREx_EnableFastWakeUp();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();

    // following should never be reached...
    putstr(" STANDBY ERROR\n");

    LL_mDelay(60000);

    HAL_DeInit();
    NVIC_SystemReset();
}


void lora_rx() {
    uint8_t buf[0xff];
    signal_t sig;
	uint32_t received = rfm95_recv(&rfm95_dev, buf, sizeof(buf), &sig);
	if( received ) {
	    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
		putstr("\ngot ");
		putul(received);
		putstr(" bytes at snr ");
		putul(sig.snr);
		putstr(" rssi ");
		putul(sig.rssi);
		putstr(": ");
		for( uint32_t pos = 0; pos < received; pos++ ) {
			puthex(buf[pos]);
		}
	    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  ///LL_GPIO_SetOutputPin(pins[BME280_CS_PIN_ID].port, pins[BME280_CS_PIN_ID].pin);
  LL_GPIO_SetOutputPin(pins[RFM95_NSS_PIN_ID].port, pins[RFM95_NSS_PIN_ID].pin);

  frame_counter = (uint16_t)HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

  putstr("\nStart ST32ML0LoraTRx 1.2 " __DATE__ " " __TIME__ " device 0x");
  uint32_t *id_ptr = (uint32_t *)0x1FF80050;
  //putlhex(*id_ptr);
  //putlhex(*(id_ptr+1));
  putlhex(*(id_ptr+5));
  payload.id = *(id_ptr+5);

  ///for( unsigned i = 0; i < 4; i++ ) {
  ///  puthex(DevAddr[i]);
  ///}

  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    putstr(" from standby");
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }
  else {
    putstr(" from reset");
    // At first power on give a button cell a bit of time
    // to charge a capacitor before sending something to prevent boot loop
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    LL_mDelay(10);
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    LL_mDelay(1000);
  }

  // After a few standby cycles don't waste power on led
  if( frame_counter <= QUIET_FRAME ) {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
  }

  rfm_setup(frame_counter);
  ///bme_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    lora_tx();
    // lora_rx();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_4);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }
  LL_RCC_EnableRTC();
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }
  LL_SetSystemCoreClock(1048000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
