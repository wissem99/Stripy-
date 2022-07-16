/*
 * WS2812.c
 *
 *  Created on: 16 juil. 2022
 *      Author: wisse
 */

#include "main.h"
#include "WS2812.h"
#include "math.h"



/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;




uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

int datasentflag=0;

// Callback for when data transfer is complete
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Green, int Red, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}



void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else

		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 58;  // 2/3 of 120
			}

			else pwmData[indx] = 28;  // 1/3 of 120

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}




// functions
// brakeFull : all leds red with 100% Brightness : 36 leds

void BrakeFull()
{
  for (int i = 0; i < MAX_LED; i++)
  {
	  Set_LED (i, 0, 255 , 0);
  }
  WS2812_Send();
}

// BrakeMidlle : led 10 to led 36-10

void BrakeMiddle()
{
  for (int i = BlinkerLEDs; i < (MAX_LED - BlinkerLEDs); i++)
  {
    Set_LED(i,0,255,0);
  }
    WS2812_Send();
}

// ParkFull : all led are red with 25% Brightness

void ParkFull()
{
  for (int i = 0; i < MAX_LED; i++)
  {
	  Set_LED(i,0,150,0);
  }
      WS2812_Send();
}

// ParkMiddle : led 10 to 36 : red with 25%brightness

void ParkMiddle()
{
  for (int i = BlinkerLEDs; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED (i,0,150,0);
	   }
	       WS2812_Send();
	 }

// Left Functions

// Led for BlinkerLEDs-1 to 0 are Blinking with delay equal to BlinkerSpeed

void LeftBlinker()
{
  for (int i = (BlinkerLEDs-1); i >= 0; i--)
  {
	  Set_LED(i,70, 255, 0);
	  WS2812_Send();
	  HAL_Delay(BlinkerSpeed) ;

  }
}

//Led for 0 to BlinkerLEDs are reset to 0

void LeftDim()
{
  for (int i = 0; i < BlinkerLEDs; i++)
  {
	  Set_LED(i,0, 0, 0);
  }
  WS2812_Send();
}

// Led for 0 to (MAX_LED - BlinkerLEDs) are set to Red color with 30% Brightness

void LeftLit()
{
  for (int i = 0; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED(i,0, 150, 0);
  }
  WS2812_Send();
}

// Led for 0 to (MAX_LED - BlinkerLEDs) are set to Red color with full Brightness

void LeftFull()
{
  for (int i = 0; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED(i,0, 255, 0);
  }
  WS2812_Send();
}

// Right Functions

//Led for (MAX_LED - BlinkerLEDs) to MAX_LED are Blinking with delay equal to BlinkerSpeed

void RightBlinker()
{
  for (int i = (MAX_LED - BlinkerLEDs); i < MAX_LED; i++)
  {

	  Set_LED(i,70, 255, 0);
	  WS2812_Send();
      HAL_Delay(BlinkerSpeed);


  }

}

//Led for (MAX_LED - BlinkerLEDs) to MAX_LED are reset to 0

void RightDim()
{
   for (int i = (MAX_LED - BlinkerLEDs); i < MAX_LED; i++)
  {
	   Set_LED(i,0, 0, 0);
  }
   WS2812_Send();
}

// Led for BlinkerLEDs to MAX_LED are set to Red color with 30% Brightness

void RightLit()
{
  for (int i = BlinkerLEDs; i < MAX_LED; i++)
  {
	  Set_LED(i,0, 150, 0);
  }
  WS2812_Send();
}

// Led for BlinkerLEDs to MAX_LED are set to Red color with full Brightness

void RightFull()
{
  for (int i = BlinkerLEDs; i < MAX_LED; i++)
  {
	  Set_LED(i,0, 255, 0);
  }
  WS2812_Send();
}

// Dual blinking

void DualBlinker()
{
  for (int i = (BlinkerLEDs-1); i >= 0; i--)
  {
	  Set_LED(i,70, 255, 0);
	  Set_LED(MAX_LED-1-i,70, 255, 0);
	  WS2812_Send();
     HAL_Delay(BlinkerSpeed);}
  }
void Start_ON (){


	for (int i = 1; i < (MAX_LED /2); i++)
	  {
		Set_LED(i,0, 60, 0);
		Set_LED(i-1,0, 0, 0);
	  Set_LED((MAX_LED -1)-i,0, 60, 0);
	    Set_LED((MAX_LED )-i,0, 0, 0);
	    WS2812_Send();
	    HAL_Delay(BlinkerSpeed);
	  }

	  for (int j = ((MAX_LED /2)-1); j >= 0; j--)
	  {
	    Set_LED(j,0, 60, 0);
	    Set_LED((MAX_LED /2-1)+((MAX_LED /2)-j),0, 60, 0);
	    WS2812_Send();
	    HAL_Delay(BlinkerSpeed);
	  }

	  for (int i = 0; i < 5; i++)
	  {
	    for (int i = 0; i < MAX_LED ; i++)
	    {
	    Set_LED(i,0, 0, 0);
	    }
	    WS2812_Send();
	    HAL_Delay (75);
	    for (int i = 0; i < MAX_LED ; i++)
	    {
	    Set_LED(i,0, 60, 0);
	    }
	    WS2812_Send();
	    HAL_Delay(50);
	  }
}

void Left(){
	LeftDim();
	RightLit();
	LeftBlinker();
	LeftDim();
	HAL_Delay(BlinkerOffDelay);

}
void Right(){
	RightDim();
	LeftLit();
	RightBlinker();
	RightDim();
	HAL_Delay(BlinkerOffDelay);

}
void Left_Break(){
	LeftDim();
	RightFull();
	LeftBlinker();
	LeftDim();
	HAL_Delay(BlinkerOffDelay);

}
void Right_Break(){
	RightDim();
	LeftFull();
	RightBlinker();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
}
void Dual(){
	LeftDim();
	RightDim();
	ParkMiddle();
	DualBlinker();
	LeftDim();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
}
void Dual_Break(){
	LeftDim();
	RightDim();
	BrakeMiddle();
	DualBlinker();
	LeftDim();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
}
