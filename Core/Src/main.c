/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * 12AUG2022		Matthew Zemlick
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "bittable.h"
#include "Chartable.h"
#include "matrixLayout.h"

#include "ssd1306.h"
#include "fonts.h"
#include "bitmap.h"
//#include "eeprom.h"

//#include "usbd_cdc_if.h" //USB virtual Com Port
//sprintf("int x= %d",x);
//CDC_Transmit_FS((uint8_t*)USBbuffer, (uint16_t)strlen(USBbuffer));

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMLEDS 168 //number of leds 128 or 168 or 121 (168 has dummy columns)
#define EXPANDFACTOR 9 // 3 bytes to SPI for 1 byte of color. green or red or blue
#define SPILOWTIME 30 //(30 for rgb3535) this is calculated on spi clock speed. it needs to be 50us of spi 0 bytes
#define BMAX 11*4//11
#define BMED 6*4//6
#define BMIN 2*4

#define numCols 21
#define numRows 8
//#define RAND_MAX 32767

#define SIMULATE 0

//Leds are G,R,B
#define BLACK 0x000000
#define GREEN 0x0A0000
#define RED 0x000A00
#define BLUE 0x00000A

#define SAMPLE_FREQ 40000//must match TIM3 Freq
#define N	64 //64
#define PI	3.14159265
#define FFT_LIB_REV 0x14
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00
#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYP_HANN 0x02 /* hann */
#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYP_NUTTALL 0x04 /* nuttall */
#define FFT_WIN_TYP_BLACKMAN 0x05 /* blackman */
#define FFT_WIN_TYP_BLACKMAN_NUTTALL 0x06 /* blackman nuttall */
#define FFT_WIN_TYP_BLACKMAN_HARRIS 0x07 /* blackman harris*/
#define FFT_WIN_TYP_FLT_TOP 0x08 /* flat top */
#define FFT_WIN_TYP_WELCH 0x09 /* welch */
#define twoPi 6.28318531
#define fourPi 12.56637061
#define sixPi 18.84955593

//magic numbers for audio stuff
#define CAL_OFFSET 70 //this may want to be found and stored during a Cal phase. (its the ADC offset for any error on the 1.65V op-amp biasing)
#define HOFFSET_SCALER 4 //this means max and min will at minimum be 2*HOFFSET_SCALER apart
#define USEDBINS 16
#define SENSERATE_H 300
#define SENSERATE_L 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
  void setPixelColor(int,uint32_t);
  uint32_t getPixelColor(int);
  void show(void);
  void getrand(int);
  long map(long, long, long, long, long);
  int32_t constrain(int32_t,int32_t,int32_t);
  void HorizShiftR(void);
  void HorizShiftL(void);
  void rotateCCW(uint32_t);
  void sine1 (uint32_t);
  void nBalls (void);
  void nGrid (void);
  void splash(void);
  uint32_t Color(uint8_t,uint8_t,uint8_t);
  void BlankScreen(uint32_t);
  void printCharWithShiftL(char, int);
  void printStringWithShiftL(char*, int);
  int randomNum(int, int);

  void getMAG();
  void graphMAG(void);
  int16_t read_adc (void);
  uint16_t read_bat_adc(void);
  void ComputeFFT(double* , double* , uint16_t , uint8_t , uint8_t);
  void ComplexToMagnitude(double* , double* , uint16_t);
  uint8_t Exponent(uint16_t);
  void Swap(double* , double*);
  void Windowing(double* , uint16_t , uint8_t , uint8_t);
  double sq(double);

  void adjustVals(void);//dumb name
  uint16_t audioMap(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

  int checkbuttons(void);
  int checkUSBConnected(void);
  void USBdataEnable(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint8_t myData[(NUMLEDS*EXPANDFACTOR)+SPILOWTIME]; //holds all the expanded Pixel Color data + the 50uS of zero time (WS2812 reset)
  uint32_t Forecolor = 0x000800;
  uint8_t Brange = BMIN; //1,6,11  //set the brightness for each chanel in getrand();
  uint8_t green, red, blue; //used in getrand
  uint8_t mbuffer[3];   //used for scroll text

  uint32_t ArrayBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //this can be local?
  int randSelect = 0;
  int cycles = 10;
  uint32_t waittime = 200;
  char myString[] = "H3llo "; //used to test USB virutal COM Port Transmit


  int n_count = 0; //indexes hgihwater count
  uint8_t n_done = 0; //flag to perform fft operations
  double REX[N]; //holds Real input and Realoutput (overwritten when windowing/computeFFT is performed
  double IMX[N]; //hold imaginary output
  double MAG[USEDBINS];//magnitude of a+bi vector
  double MAGmax[USEDBINS];
  double MAGmin[USEDBINS];
  double MAGavg[USEDBINS]; //used in SIMULATE
  #define F_WIDTH 8 //number of samples. bigger = smoother but more delay
  #define F_WIDTHF 8.0f //must match above define
  double MAGbuff[USEDBINS][F_WIDTH];
  double MAGbuffSum[F_WIDTH];
  uint8_t fCount = 0; //LowPass Filter Summing indexer

  //for displaying
  int32_t MAGI[USEDBINS];//magnitude of a+bi vector used for displaying
  int32_t MAGImin[USEDBINS];//magnitude of a+bi vector used for displaying
  int32_t MAGImax[USEDBINS];//magnitude of a+bi vector used for displaying

  //N= 256, SAMPLE RATE = 30kHz
  //#define FFT_RAW_MAX 150000.0 // this is found expirementally using the SIMULATE 1 and the max ADC value possible

  //N=64, SAMPLE RATE = 20kHz
  #define FFT_RAW_MAX 37000.0 // this is found expirementally using the SIMULATE 1 and the max ADC value possible
  double fft_Db_MAX = -20.0*log10(1.0/FFT_RAW_MAX);// = 103.52182518111363

  //const int tones[6] = {1, 1, 2, 2, 1 ,2}; //these are like pitches
  //const int tonedelay[6]={1,1,1,1,1,1}; //these are duration delays

  uint16_t batteryLevel = 0;
  volatile uint8_t updateFlag = 0;
  uint32_t senseCounterH = 0;
  uint32_t senseCounterL = 0;

  //EEPROM stuff
  /* Virtual address defined by the user: 0xFFFF value is prohibited */
  //uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
  //uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
  //uint16_t VarValue = 0;
  #if SIMULATE
  double signalFrequency = 1; //FFT testing using simulated sinwave
  #endif

  //PB1 CLK / (Prescaler*Count) = Freq
  //72E6/((P+1)*(COUNT+1)) = ISR rateHz
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  	updateFlag=1;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //SETUP
      //Power INIT
    HAL_Delay(500); //wait .5 sec before power on. prevents false positives
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET);//SoftPower ON
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET);//SoftPower OFF

    //REQUIRED to set all the end frame bytes to zero (50uS ws2812 reset)
    //use memset() below
    for (int x=(NUMLEDS*EXPANDFACTOR); x<(NUMLEDS*EXPANDFACTOR)+SPILOWTIME; x++) myData[x] = 0; //ToDo: replace with memset(). why not start at 0?
    for (int i=0; i<USEDBINS; i++){ //zeroing out a bunch of arrays
  #if SIMULATE
  	  MAGmax[i] = 0;
  	  MAGmin[i] = 32000;
  #else
  	  //MAGmax[i] = (int32_t)((fft_Db_MAX/2)+HOFFSET_SCALER);
  	  //MAGmin[i] = (int32_t)((fft_Db_MAX/2)-HOFFSET_SCALER);
  	  MAGmax[i] = 32.f; //think these should be played with. wait a few mins of SILENCE and breakpoint try 30.0f
  	  MAGmin[i] = 0.0f; //try 15.0f
  #endif
  	  MAGavg[i] = 0.f;
  	  for(int k=0; k< F_WIDTH; k++) MAGbuff[i][k] = 0.0f;
  	  MAGbuffSum[i] = 0;
    }


  //Oled Init and Boot
  while(!SSD1306_Init());  // initialize. blocking if OLED not detected
  SSD1306_DrawBitmap(0,0,Boot, 128, 32, 1); //boot splash screen
  SSD1306_UpdateScreen(); //display
  HAL_Delay(1000);
  SSD1306_Clear(); //clear oled display buffer
  SSD1306_UpdateScreen();

  HAL_ADC_Start(&hadc1); //starts the adc configured in continuous mode (cubeMX)
  HAL_ADC_Start(&hadc2); //start adc2 in continuous mode
  HAL_TIM_Base_Start_IT(&htim3); //start Timer 3. used for FFT sampling
  //HAL_Delay (500);

  //printStringWithShiftL(" STARIOS GEAR 2020   ", 10); //Send Lscrolling Text (send car array)
  BlankScreen(BLACK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(updateFlag==1){ //30kHz Timer
			updateFlag = 0;
			if (!n_done){ //I need N ADC Samples
	#if SIMULATE
			//FFT testing using simulated sinwave
			const double samplingFrequency = SAMPLE_FREQ;
			const double amplitude = 4095; //4095 would be
			double cycles = (((N-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
			REX[n_count] = ((amplitude * (sin((n_count * (twoPi * cycles)) / N))) / 2.0);/* Build data with positive and negative values*/
			IMX[n_count] = 0;
			//FFT testing using simulated sinwave
	#else
				REX[n_count] = (double) read_adc() - 2048 + CAL_OFFSET;//12bit ADC (just store this into the REX[i] array. no need to have the adc_value variable
				IMX[n_count] = 0;
				#endif
				n_count++;
				if (n_count >= N){
	#if SIMULATE
				  signalFrequency+=40;//FFT testing using simulated sinwave
				  if(signalFrequency>SAMPLE_FREQ/2) signalFrequency = 1; //FFT testing using simulated sinwave
	#endif
				n_done = 1;
				n_count = 0;
			  }
			}
	    }
		if(n_done==1){
			memset(IMX,0,N*sizeof(double)); //zero out complex part before FFT
			Windowing(REX, N, FFT_WIN_TYP_HANN, FFT_FORWARD);		//HANN	//Apply Window the the ADC data
			ComputeFFT(REX, IMX, N, Exponent(N),FFT_FORWARD); 	//perform the exponent in setup to optimize
			getMAG(); // compute the magnitude of 16 FFT samples
			adjustVals(); //autoscaling. max sensing
			graphMAG(); //graph the complex magnitudes on the oled and RGB Display
			n_done = 0;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int checkbuttons(void){ // BTN3 = PB5 needs internal Pullups enabled
	int c = 0;
	c |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)<<2 | HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)<<1 | HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
	return c;
}

int checkUSBConnected(void){ // Check for USB is plugged in to power
	int c = 0;
	c=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	return c;
}

void USBdataEnable(int c){ //this function will enable USB serial device if VBUS is connected
	if(c){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET);//pull USB Data+ (DP) High
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET);//pull USB Data+ (DP) LOW
	}
}

////////////////////////////////////////////////////////////////////////////////START FFt Lib functions
void ComputeFFT(double *vReal, double *vImag, uint16_t samples, uint8_t power, uint8_t dir)
{	// Computes in-place complex-to-complex FFT
	// Reverse bits
	uint16_t j = 0;
	for (uint16_t i = 0; i < (samples - 1); i++) {
		if (i < j) {
			Swap(&vReal[i], &vReal[j]);
			if(dir==FFT_REVERSE)
				Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	// Compute the FFT
	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;
	for (uint8_t l = 0; (l < power); l++) {
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			 for (uint16_t i = j; i < samples; i += l2) {
					uint16_t i1 = i + l1;
					double t1 = u1 * vReal[i1] - u2 * vImag[i1];
					double t2 = u1 * vImag[i1] + u2 * vReal[i1];
					vReal[i1] = vReal[i] - t1;
					vImag[i1] = vImag[i] - t2;
					vReal[i] += t1;
					vImag[i] += t2;
			 }
			 double z = ((u1 * c1) - (u2 * c2));
			 u2 = ((u1 * c2) + (u2 * c1));
			 u1 = z;
		}
		c2 = sqrt((1.0 - c1) / 2.0);
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
		c1 = sqrt((1.0 + c1) / 2.0);
	}
	// Scaling for reverse transform
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < samples; i++) {
			 vReal[i] /= samples;
			 vImag[i] /= samples;
		}
	}
}

//not used
void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples){	// vM is half the size of vReal and vImag
	for (uint16_t i = 0; i < samples; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}

uint8_t Exponent(uint16_t value){
	// Calculates the base 2 logarithm of a value
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}


void Swap(double *x, double *y){
	double temp = *x;
	*x = *y;
	*y = temp;
}

//commented on unused Windowing types so save flash space
void Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
{// Weighing factors are computed once before multiple use of FFT
// The weighing function is symetric; half the weighs are recorded
	double samplesMinusOne = (double)(samples - 1.0);
	for (uint16_t i = 0; i < (samples >> 1); i++) {
		double indexMinusOne = (double) i;
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		// Compute and record weighting factor
		switch (windowType) {
//		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
//			weighingFactor = 1.0;
//			break;
		case FFT_WIN_TYP_HAMMING: // hamming
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: // hann
			//weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
//		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
//			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
//			break;
//		case FFT_WIN_TYP_NUTTALL: // nuttall
//			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN: // blackman
//			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
//			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
//			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_FLT_TOP: // flat top
//			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
//			break;
//		case FFT_WIN_TYP_WELCH: // welch
//			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
//			break;
		}
		if (dir == FFT_FORWARD) {
			vData[i] *= weighingFactor;
			vData[samples - (i + 1)] *= weighingFactor;
		}
		else {
			vData[i] /= weighingFactor;
			vData[samples - (i + 1)] /= weighingFactor;
		}
	}
}


double sq(double x){ //math squaring function
return 	x*x;
}


//added an offset of 1.68V on the ADC pin. this comes out to 2048 for a 12-bit ADC
//ToDo: this signed val and the offset should be done in the mainloop. this will allow the read_adc to be generalized. should return a uint16_t from the 12-bit adc
int16_t read_adc(){ //reads ADC0 into adcVAL (12-bit?)
    HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100); //wait (blocking) for the conversion to complete ?
    //because HAL_ADC_Getvalue only returns 0 to 2^12-1; half of that is 2047.5. lets round it to 2048 (this is fine for a int16_t)
	int16_t adcVal = (int16_t)(HAL_ADC_GetValue(&hadc1) & (0xFFFF)); //make sure only the lower 16 bytes are stored.
    HAL_ADC_Stop(&hadc1);
	return adcVal;
}
////////////////////////////////////////////////////////////////////////////////END FFt Lib functions


//ToDo: remove this adc. just genrealize the read_adc() to accept a ADC_HandleTypeDef. can pass in adc1 or adc2
uint16_t read_bat_adc(){ //reads ADC0 into adcVAL (12-bit?)
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);//Battery Check Enabled
  HAL_Delay(10);
  //HAL_ADC_Start(&hadc2);
  uint16_t adcBatVal = HAL_ADC_GetValue(&hadc2);
 // HAL_ADC_Stop(&hadc2);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_RESET);//Battery Check Disabled
return adcBatVal;
}




//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA Audio Adjustments
#if SIMULATE
void getMAG(){
	uint8_t indexOffset = 1; //the lowest used bin index REX[0] contains alot of DC
	for (uint32_t i = 0; i < USEDBINS; i++){
		uint32_t idx = (i*(N/32))+indexOffset; //used to make sure i am indexing the REX[] properly
		MAG[i] = (sqrt(sq(REX[idx]) + sq(IMX[idx]))); //Magnitude from Pythagoram of Real and Complex in dBs
	}
}
#else
void getMAG(){
	uint8_t indexOffset = 1; //the lowest used bin index
	double MaxTemp;
	uint32_t idx;
	for (uint32_t i = 0; i < USEDBINS; i++){
		idx = (i*(N/32))+indexOffset; //used to make sure i am indexing the REX[] properly
		//Todo: peak detection here?

		MaxTemp = sqrt(sq(REX[idx]) + sq(IMX[idx])); //Magnitude from Pythagoram of Real and Complex
		MAG[i] = (double) fft_Db_MAX + (20*log10((MaxTemp+1.0)/FFT_RAW_MAX)); //return positive DBs. this will be used
	}
}
#endif
////with N = 256, Only 128 Bins are usable
//void getMAG(){ //testing a multiple bin method. its basically using a half (right side only window)
//#define width 8
//#define ratio 1
//	uint8_t a=0;
//	uint8_t b=0;
////int msgfreqs[7] = {63, 160, 400, 1000, 2500, 6250, 16000};
////int msgfreqs[7] = {16.64, 26.17, 41.17, 64.76, 101.88, 160.25, 252.07, 396.51, 623.71, 981.10, 1543.27, 2427.56, 3818.55, 6006.57, 9448.34, 14862.24};
////int msgfreqsIDX[16];
//	uint8_t indexOffset = 2; //the lowest used bin index
//	for (uint32_t i = 0; i < USEDBINS; i++){
//		uint32_t idx = (i*(USEDBINS/2))+indexOffset; //used to make sure i am indexing the REX[] properly
//
//		MAG[i] = 0;
//		for(int k = 0; k<width; k++){ //trying to use 6 samples to spread out the freq response
//			if(k+idx<(N/2)-width){
//				a=1;
//				MAG[i] += (int32_t) ((sqrt(sq(REX[idx+k]) + sq(IMX[idx+k])))); //Magnitude from Pythagoram of Real and Complex
//			}else{
//				a=0;
//			}
//			if(k+idx>width){
//				b=1;
//				MAG[i] += (int32_t) ((sqrt(sq(REX[idx-k]) + sq(IMX[idx-k]))));
//			}else{
//				b=0;
//			}
//		}
//		MAG[i] /= width*(a+b);
//
//		//testing log function
//		//MAG[i] = (int32_t)(20*(log10(MAG[i]+10)-1));
//	}
//}

//thinking of making a correlating function. gaussian distribution 'window' e^-(x-n)^2 to the magnitudes.
//I could use desmos to get like 8 sample points. multiply, add them up, divide by n so I dont have to waste CPU on math.
//The idea is to 'widen' the each bin's bandwidth
//void getMAG(){
//	uint8_t indexOffset = 2; //the lowest used bin index
//	for (uint32_t i = 0; i < USEDBINS; i++){
//		uint32_t idx = (i*(USEDBINS/2))+indexOffset; //used to make sure i am indexing the REX[] properly
//		MAG[i] = (int32_t) (sqrt(sq(REX[idx]) + sq(IMX[idx]))); //Magnitude from Pythagoram of Real and Complex
//		//testing log function
//		//MAG[i] = (int32_t)(20*(log10(MAG[i]+10)-1));
//	}
//}
#if SIMULATE
void adjustVals(){ //get 16 FFT manitudes. each bin is 30kHz/N
	for (uint32_t i = 0; i < USEDBINS; i++){
		MAGavg[i] = (MAGavg[i] + MAG[i])/2; //store the avg
		if(MAG[i] > MAGmax[i]) MAGmax[i] = MAG[i]+1; //store the Max
		if(MAG[i] < MAGmin[i]) MAGmin[i] = MAG[i]-1; //store the Min

		if(MAGmin[i]<1){ //shitty code to make sure that MAGmin is above zero and MAG is 1 above the min
			MAGmin[i] = 0;
		}
		MAGI[i] = (int32_t) MAG[i]; //store the integer for display
		MAGImin[i] = (int32_t) MAGmin[i]; //store the integer for display
		MAGImax[i] = (int32_t) MAGmax[i]; //store the integer for display
	}
}

#else
//ToDo: clean up these for-loops. sense counter must be outside a for-loop(maybe theres a way...)
//Todo: anyway to speed this funtion up?
void adjustVals(){ //main audio filtering work

	//////lowpass filtering//////
	for (uint32_t i = 0; i < USEDBINS; i++){
		MAGbuff[i][fCount] = MAG[i]; //store the current reading into the circular buffer
		for (uint32_t k = 0; k < F_WIDTH; k++){ //store the sums
			MAGbuffSum[i] += MAGbuff[i][k];
		}
		MAG[i] = MAGbuffSum[i]/F_WIDTHF;
		MAGbuffSum[i] = 0;
	}
	fCount++; //lowpass filter indexer
	if(fCount>=F_WIDTH) fCount=0;


	//////max and min storing//////
	for (uint32_t i = 0; i < USEDBINS; i++){
		if(MAG[i] > MAGmax[i]){
			  MAGmax[i] = MAG[i]; //store the Max
			  senseCounterH = 0;
		}
		if(MAG[i] < MAGmin[i]){
			  MAGmin[i] = MAG[i]; //store the Max
			 // senseCounterL = 0;
		}
	}


	////// max and min auto adjusting//////
		  senseCounterH++;
		  senseCounterL++;
		  if(senseCounterH>SENSERATE_H){ //about every 3 seconds
			  senseCounterH = 0;
			  for (uint32_t k = 0; k < USEDBINS; k++){
				  if(MAGmax[k]>MAGmin[k] + HOFFSET_SCALER) MAGmax[k]-=0.5f; //reduce upper limit
			  }
		  }
		  if(senseCounterL>SENSERATE_L){ //about every 1 seconds
			  senseCounterL = 0;
			  for (uint32_t k = 0; k < USEDBINS; k++){
				  if(MAGmin[k]<MAGmax[k] - HOFFSET_SCALER) MAGmin[k]+=0.5f; //increase lower limit
			  }
		  }

//not sure thus is needed
	//////make sure that MAG is within min and max as a double//////
		  for (uint32_t i = 0; i < USEDBINS; i++){
			  if(MAG[i]>=MAGmax[i]) MAG[i] = MAGmax[i]-1;
			  if(MAG[i]<=MAGmin[i]) MAG[i] = MAGmin[i]+1;
			  //MAG[i] = constrain(MAG[i],MAGmin[i]+1,MAGmax[i]-1); //instead?
		  }


	//////Typecasting to ready for digital displaying//////
		  for (uint32_t i = 0; i < USEDBINS; i++){
				MAGI[i] = (int32_t) MAG[i];
				MAGImin[i] = (int32_t) MAGmin[i];
				MAGImax[i] = (int32_t) MAGmax[i];
			}


	//////make sure mag is within min and max as a integer//////
		  for (uint32_t i = 0; i < USEDBINS; i++){
			  if(MAG[i]>=MAGmax[i]) MAG[i] = MAGmax[i]-1;
			  if(MAG[i]<=MAGmin[i]) MAG[i] = MAGmin[i]+1;
		  }
}
#endif

uint16_t audioMap(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) { //Map function spacial
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//int8_t rgbMap(uint32_t x, uint32_t in_min, uint32_t in_max, uint8_t out_min, uint8_t out_max) { //Map function color
//rDist = audioMap(c,red,0);	 // valS[x] proportional to nLength
//gDist = audioMap(c,green,0);	 // valS[x] proportional to nLength
//bDist = audioMap(c,blue,0);	 // valS[x] proportional to nLength
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}


//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA Audio Adjustments


void graphMAG() { //draws FFT magnitudes MAG[0] to MAG[15] on OLED and RGB Matrix
	SSD1306_Clear();
#if SIMULATE
  	SSD1306_Puti(127-(7*5), 0, signalFrequency, 5);
#endif
  for(int k=0; k<USEDBINS; k++) SSD1306_DrawFilledRectangle(8*k, (uint16_t)((SSD1306_HEIGHT-1)-abs(audioMap(MAGI[k],MAGImin[k],MAGImax[k],0,SSD1306_HEIGHT-1))), 6, (uint16_t)(abs(audioMap(MAGI[k],MAGImin[k],MAGImax[k],0,SSD1306_HEIGHT-1))), SSD1306_COLOR_WHITE); //draw 16 bars abs(x[k]) on (0 to k/2)
  SSD1306_UpdateScreen(); //display //this may be calling the display too much
  int k = 5;
  BlankScreen(BLACK);
  for(int x=0; x<USEDBINS; x++){ //graph on rgb glasses. mirrored. only showing lower 8 freqs.
	  if(MAG[0] > MAGmax[0]*0.9) getrand(randSelect); //Careful, This is using the double. if the lowest used freq hits 75% of its make then change colors.
	  for(uint16_t y=0; y<audioMap(MAGI[x],MAGImin[x],MAGImax[x],0, numRows); y++){
		if(MAGI[x]>MAGImin[x]+HOFFSET_SCALER){//only display if greater than MAGmin + 4;
		  if(x<8){
			  setPixelColor(ColumnArray[x][y], Forecolor);   //I changed this to work with the 168 led display(21 cols) and 32 (64/2 usable) Bins
		  //setPixelColor(ColumnArray[numCols-1-x][y], Forecolor); //mirror but x has to count to USEDBINS/2
		  }else{
			  setPixelColor(ColumnArray[x+k][y], Forecolor);   //I changed this to work with the 168 led display(21 cols) and 32 (64/2 usable) Bins
		  }
		}

	  }
  }
  show();
}


void splash(){
BlankScreen(BLACK);
for(int c=0; c<15; c++){
getrand(1); //random red color
setPixelColor( rand() % (NUMLEDS - 0 + 1) + 0, Forecolor); //add a sprinkle                                 //make sprikle brighntess propotional to the freq amplitude
getrand(2); //random red color
setPixelColor( rand() % (NUMLEDS - 0 + 1) + 0, Forecolor); //add a sprinkle
getrand(3); //random red color
setPixelColor( rand() % (NUMLEDS - 0 + 1) + 0, Forecolor); //add a sprinkle
}
show();
}


void show(void){
HAL_SPI_Transmit_DMA(&hspi2, myData, (NUMLEDS*EXPANDFACTOR)+SPILOWTIME); //Begin SPI - SPI DMA data burst to LEDS
}


//ToDo: testing LUT vs Algorith to generate SPI data. I can save 1kb of flash text space by using a algorith instead of a LUT
//Todo: All the RGB SPI stuff should be in a class. (pixel timing type, number of pixels, brightness?, display buffer, funtions)
void setPixelColor(int pixelNum, uint32_t c){ //pass in a Pixel Number and 32bit Color and map to 9 bytes
	  uint8_t myGRB[3]; //create an array to hold the GRB bytes for one 23bit color
	  for(int x=0; x<3; x++) myGRB[x] = (c >> ((2-x) * 8)) & 0xFF; //extract the green,red,blue from the 32bit and write the 8bit values into myGRB array
	  for(int y=0; y<3; y++){ //this is to index the G,R,B bytes
	    for(int x=0; x<3; x++)  myData[(y*3)+(pixelNum*9)+x] = (bitExpand[myGRB[y]] >> ((2-x) * 8)) & 0xFF; //expand green, red, blue. from 1 byte into 3 bytes each (9 bytes total)
	  }
}

uint32_t getPixelColor(int c){ //receives a Led Number and returns its 32 bit value
	uint8_t myGRB[3];
	uint32_t myExpanedByte[3]; //
	uint8_t i=0; //used to index the bittable
	for(int x=0; x<3; x++) myExpanedByte[x] =  ((uint32_t)myData[(c*9)+(3*x)] << 16) | ((uint32_t)myData[(c*9)+(3*x)+1] <<  8) | (uint32_t) myData[(c*9)+(3*x)+2]; //extract values from myData Array
    for(int x=0; x<3; x++){
	  while(myExpanedByte[x] != bitExpand[i]) i++; //loop until match occurs
	  myGRB[x] = i; //store the index into 3 bytes sized values
	  i=0;
     }
	return ((uint32_t)myGRB[0] << 16) | ((uint32_t)myGRB[1] <<  8 | (uint32_t) myGRB[2]); //return a uint32_t value for the color stored on the led
}



void BlankScreen(uint32_t c) { //quickly set the entire screen one color
  for (int i = 0; i < NUMLEDS; i++) {
    setPixelColor(i, c);
  }
  show();
}


void rotateCCW(uint32_t theta){ //rotates all pixels theta rads counter clockwise //not quite working yet. something with math is wrong. test this in codeblocks?
	int myOffset = 5;
	int myX = 10;
	int myY = 5;
	//move to origin: x-
	for(float i=0; i<=2; i+=.5){
	int myXT = myOffset + (int) ((myX-myOffset)*cos(M_PI*i)-(myY-myOffset)*sin(M_PI*i));
	int myYT = myOffset + (int) ((myX-myOffset)*sin(M_PI*i)+(myY-myOffset)*cos(M_PI*i));
	BlankScreen(0);
	setPixelColor(ColumnArray[myX][myY], 0x0A0000); ///set a pixel green
	show();
	HAL_Delay(1000);
	setPixelColor(ColumnArray[myXT][myYT], 0x000A00);
	show();
	HAL_Delay(1000);
	myX = myXT;
	myY = myYT;
	}
}

//shift right function
void HorizShiftR () {   //Shift columns right 1 palce
  for ( int s = numCols-2; s >= 0; s--) { //read one hind. index COLUMNS
    for (int b = 0; b <= numRows-1; b++) { //read each Led value.  inded the ROWS
      ArrayBuffer[b] = getPixelColor(ColumnArray[s][b]); //store 5 int32s into bufferarray
      setPixelColor(ColumnArray[s + 1][b], ArrayBuffer[b]); //put those stored vals in place one to the right
    }
  }
  show();
}

//shift Left function
void HorizShiftL () {   //Shift columns right 1 palce
  for ( int s = 1; s <= numCols-1; s++) { //read one behind left
    for (int b = 0; b <= numRows-1; b++) { //read each Led value
      ArrayBuffer[b] = getPixelColor(ColumnArray[s][b]); //store 5 int32s into bufferarray
      setPixelColor(ColumnArray[s - 1][b], ArrayBuffer[b]); //put those stored vals in place one to the left
    }
  }
  show();
}

void BlankColumn(int c) {    //wipes the coulmn 0 to clear out jjunk, to get ready for shifting
  for (int w = 0; w <= numRows-1; w++) {
   setPixelColor(ColumnArray[c][w], 0x000000);
  }
}

//Extract characters for Scrolling text
void printStringWithShiftL(char* s, int shift_speed) { //Add Color??
	while (*s != 0) {
		printCharWithShiftL(*s, shift_speed);
		s++;
	}
}

// Put extracted character on Display     // this works very well
void printCharWithShiftL(char c, int shift_speed) {
	enum {TEXTPOS = 1};
	if (c < 32) return; //error check for ascii values less than ASCII  < ' ' (space) = 32 >
	c -= 32; //align the char ascii with the CharTable
	memcpy(mbuffer, &CH[3 * c], 3); //CH + 3*c is same as &CH[3*c] //copy 3 items
	//mBuffer
	getrand(randSelect);
	for (int j = 0; j <= 2; j++) {
		uint8_t b = 0b00000001;
		for (int k = TEXTPOS; k < TEXTPOS+5; k++) {
			if (mbuffer[j]&b) setPixelColor(ColumnArray[numCols-1][(numRows-1)-k], Forecolor);
			b = b << 1;
		}
		show();
		HorizShiftL(); //shift every column one column left (one space between letters)
		BlankColumn(numCols-1);
		HAL_Delay(shift_speed);
	}
	HorizShiftL(); // (one space after word)

}


uint32_t Color(uint8_t g,uint8_t r,uint8_t b){ //receives a G,R, amd B byte then Returns a 32bit int
return ((uint32_t)g << 16) | ((uint32_t)r <<  8 | (uint32_t) b);
}


long map(long x, long in_min, long in_max, long out_min, long out_max) { //Map function
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t constrain(int32_t x, int32_t min, int32_t max){
	if(x < min) return min;
	if(x > max) return max;
	return x;
}

int randomNum(int myLow, int myHigh){  //return random integer for low to high range
	return  (rand() % (myHigh - myLow + 1)) + myLow;
}


void sine1 (uint32_t delayTime) { //single wave  //Shift columns right 1 palce
  for (int i = 0; i < numRows; i++) { //number here determins "density"
			BlankColumn(0);
			setPixelColor(ColumnArray[0][i], Forecolor);
			HorizShiftR();
			HAL_Delay(delayTime);
  }
  for (int i = 0; i < numRows; i++) { //number here determins "density"
			BlankColumn(0);
			setPixelColor(ColumnArray[0][(numRows-1) - i], Forecolor);
			HorizShiftR();
			HAL_Delay(delayTime);
  }
}

//Todo: make this an object
void nGrid(){ //make a snake head pop upin a random spot and move it arround the screen randomly
	//BlankScreen(0);
	int numBalls = randomNum(4,7); //number of balls
	int Limitx = numCols-1; //width
	int Limity = numRows-1;//height
	int Headx[numBalls]; //head postion holder
	int Heady[numBalls]; //head postion holder
	int Dirx[numBalls]; //-1 or 1
	int Diry[numBalls]; //-1 or 1
	int Tailx[numBalls]; //tailx psotion. is not really 'seen'. exists to reduce "flickering"
	int Taily[numBalls]; //taily position
	int myGreen[numBalls];
	int myRed[numBalls];
	int myBlue[numBalls];
	for(int c=0; c<numBalls; c++){ //loop through all arrays to set intials
	  getrand(randSelect);
	  myGreen[c] = green; //n number if random colors
	  myRed[c] = red;
	  myBlue[c] = blue;
	  Headx[c] = randomNum(0,Limitx+1); //pick n number of random x start points
	  Heady[c] = randomNum(0,Limity+1); //pick n number of random y start points
	  if(randomNum(0,2)){
	  Dirx[c] = 1;//pow(-1,random(1,2));
	  Diry[c] = 1;//pow(-1,random(1,2));
	  }else{
	  Dirx[c] = -1;//pow(-1,random(1,2));
	  Diry[c] = -1;//pow(-1,random(1,2));
	  }
	}
    for(int i=0; i<=cycles; i++){
		 //checkbtn3();
		  for(int c=0; c<numBalls; c++){
			   if(Headx[c]==0) Dirx[c]=1;
			   if(Headx[c]==Limitx) Dirx[c]=-1;
			   if(Heady[c]==0) Diry[c]=1;
			   if(Heady[c]==Limity) Diry[c]=-1;

			  Tailx[c] = Headx[c];
			  Taily[c] = Heady[c];
			  if(c%2==0){
			  Headx[c]+=Dirx[c];
			  }else{
			  Heady[c]+=Diry[c];
			  }
			  setPixelColor(ColumnArray[Headx[c]][Heady[c]],Color(myGreen[c]/(numBalls-c),myRed[c]/(numBalls-c),myBlue[c]/(numBalls-c))); //set the pixel
			  setPixelColor(ColumnArray[Tailx[c]][Taily[c]],0); //clear the pixel (causing flashing?
		}
		    show();
		 	HAL_Delay(waittime);
	}
	if(randomNum(0,1000)==500){
	printStringWithShiftL(" STARIOS", 40); //Send Lscrolling Text (send car array)
	}

}

//Todo: make this an object
void nBalls(){ //make a snake head pop upin a random spot and move it around the screen randomly
	//rand usage: int myRandomnumber = (rand() % (MAX - MIN + 1)) + MIN;
#define MAXBALLS 20
#define MINBALLS 8
	int numBalls = (rand() % (MAXBALLS - MINBALLS + 1)) + MINBALLS; //number of balls
	//int Limitx = (numCols-1); //width
	//int Limity = (numRows-1);//height
	int Headx[numBalls]; //head potion holder
	int Heady[numBalls]; //head postion holder
	int Dirx[numBalls]; //-1 or 1
	int Diry[numBalls]; //-1 or 1
	int Tailx[numBalls]; //tailx position. is not really 'seen'. exists to reduce "flickering"
	int Taily[numBalls]; //taily position
	int myGreen[numBalls];
	int myRed[numBalls];
	int myBlue[numBalls];
	for(int c=0; c<numBalls; c++){ //loop through all arrays to set intials
	  getrand(randSelect);
	  myGreen[c] = green; //n number if random colors
	  myRed[c] = red;
	  myBlue[c] = blue;
	  Headx[c] = (rand() % ((numCols-1) - 0 + 1)) + 0; //pick n number of random x start points //could be replaced using my randomNum function
	  Heady[c] = (rand() % ((numRows-1) - 0 + 1)) + 0; //pick n number of random y start points
	  Dirx[c] = 1;//pow(-1,random(1,2));
	  Diry[c] = 1;//pow(-1,random(1,2));
	}
    for(int i=0; i<=cycles; i++){
		 //checkbtn3();
		  for(int c=0; c<numBalls; c++){
			  if(Headx[c]==0) Dirx[c]=1;
			  if(Headx[c]==(numCols-1)) Dirx[c]=-1;
			  if(Heady[c]==0) Diry[c]=1;
			  if(Heady[c]==(numRows-1)) Diry[c]=-1;

			  Tailx[c] = Headx[c];
			  Taily[c] = Heady[c];
			  Headx[c]+=Dirx[c];
			  Heady[c]+=Diry[c];
			  setPixelColor(ColumnArray[Headx[c]][Heady[c]],Color(myGreen[c],myRed[c],myBlue[c])); //set the pixel
			  setPixelColor(ColumnArray[Tailx[c]][Taily[c]],0); //clear the pixel (causing flashing?
		  }
		  show();
		  HAL_Delay(waittime);
		   if((rand() % (30 - 0 + 1)) + 0 == 15){ //get a random number after all the balls have been updated
			 for(int x=0; x<numBalls; x++){ //loop through each ball
			  getrand(randSelect);
			  myGreen[x] = green; //n number if random colors
			  myRed[x] = red;
			  myBlue[x] = blue;
			  if(Headx[x]<numCols-2 && Headx[x]>=2){
				 for(int z=0; z<numBalls; z++) setPixelColor(ColumnArray[Headx[x]][Heady[x]],0); //needed to clear the stuck balls
				 if((rand() % (2 - 0 + 1)) + 0){ //add a shift to keep things from repeating too much
					 Tailx[x] = Headx[x];
					 Headx[x]++;
				 }else{
					 Tailx[x] = Headx[x];
					 Headx[x]--;
				 }
			  }
			 }
		}
	}
	for(uint8_t k=0; k<16; k++){ //picked 5 times as random amount should be enough to get all to zero
	//instead of setting the color to 0, fade it to 0. need to get the rgb for each color in the array. int32 to 3 byte
	int decayRate = 4; //used to set the speed of the dimming effect
	for(uint8_t j=0; j<numBalls; j++){
	    myRed[j]-=decayRate;
		myGreen[j]-=decayRate;
		myBlue[j]-=decayRate;
		if(myRed[j]<decayRate)myRed[j]=0;
		if(myGreen[j]<decayRate)myGreen[j]=0;
		if(myBlue[j]<decayRate)myBlue[j]=0;
		setPixelColor(ColumnArray[Headx[j]][Heady[j]],Color(myRed[j],myGreen[j],myBlue[j])); //clear the pixels this way insteadof BlankScreen
		show();
		//HAL_Delay(1);
	 }


	}
	if(rand() % (10 - 0 + 1) + 0==5){
	getrand(randSelect);
	BlankScreen(Forecolor);
	}

}


void getrand(int c) { // trying to make it //the goal is to get a random forecolor that is not white, then find the opposite of
  //rand usage: int myRandomnumber = (rand() % (MAX - MIN + 1)) + MIN;
	//getrand requries 0,1,2,3, or specify what type of color to return
      switch ((rand() % (2 - 0 + 1)) + 0) { //0 to 3 //map(rand(),0,32767,0,2)      //could be replaced using my randomNum function
        case 0:                                 //multi
            green = (uint8_t) 2*((rand() % ((Brange+1) - (1) + 1)) + 0);
            red = (uint8_t) 2*((rand() % ((Brange+1) - (0) + 1)) + 0);
            blue = 0;
            if(green==0 && red==0) green = 1;
          break;

        case 1:
        	 green = (uint8_t) 2*((rand() % ((Brange+1) - (0) + 1)) + 0);
        	 blue = (uint8_t) 2*((rand() % ((Brange+1) - (1) + 1)) + 0);
        	 red = 0;
        	 if(green==0 && blue==0) blue = 1;
          break;

        case 2:
        	 blue = (uint8_t) 2*((rand() % ((Brange+1) - (0) + 1)) + 0);
        	 red = (uint8_t) 2*((rand() % ((Brange+1) - (1) + 1)) + 0);
        	 green = 0;
        	 if(blue==0 && red==0) red = 1;
          break;
      }

  Forecolor = ((uint32_t)green << 16) | ((uint32_t)red <<  8) | ((uint32_t)blue);
  //CDC_Transmit_FS(Forecolor, 4); //prints color value to USB virtual serial port



}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
