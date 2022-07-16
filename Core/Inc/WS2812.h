/*
 * WS2812.h
 *
 *  Created on: 16 July 2022
 *  Author    : YAAKOUBI WISSEM
 *  Function  : Driver of RGB Strip LED WS2812
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_
//----------Global Variable-------------//

#define MAX_LED 29                                        // Number of LEDs in  Strip
#define USE_BRIGHTNESS 0                                  // 0 If you don't use brightness function | 1 If you don't use brightness function
#define BlinkerLEDs 7                                     // Number of LEDs For Blinking
#define HIGH_Duty 56                                      //  0.7 Duty cycle ( 56 = 0.7 * 90 (ARR for Clock 72Mhz) )
#define LOW_Duty 28                                       //  0.3 Duty cycle ( 28 = 0.7 * 90 (ARR for Clock 72Mhz) )
#define BlinkerOffDelay 300                               //Blinker Off time. Adjust this to match with your Bike blinker speed.
#define BlinkerSpeed 50                                   //Blinker Running LED Speed. Adjust this to match with your Bike blinker speed.
#define PI 3.14159265

//-------Global Functions--------------//

void Set_LED(int LEDnum, int Green, int Red, int Blue) ;  // SET LED number "LEDnum" to RGB value (G,R,B)
void Reset_LED (void) ;									  // RESET ALL LEDs
void Set_Brightness (int brightness) ;                    // Set brightness of LEDs from 0 to MAX 45
void WS2812_Send (void) ;                                 // Send data to WS2812

void Start_ON (); 										  // Start Animation
void Left();                                              // Turn Left
void Right();                                             // Turn Right
void Left_Break();										  // Turn Left  with Break
void Right_Break();										  // Turn Right with Break
void Dual();                                              // Dual Right and Left Blinking
void Dual_Break();                                        // Dual Right and Left Blinking with Break


//-------Elementary Function ---------//

void BrakeFull();                                         // All LEDs are Set Max Value Of Red Color (0,255,0)
void BrakeMiddle();                                       // LEDs From < BlinkerLEDs > To < MAX_LED - BlinkerLEDs > are Set Max Value Of Red Color (0,255,0)
void ParkFull();                                          // All LEDs are Set Average Value Of Red Color (0,150,0)
void ParkMiddle();										  // LEDs From < BlinkerLEDs > To < MAX_LED - BlinkerLEDs > are Set Average Value Of Red Color (0,150,0)
void LeftBlinker();                                       // LEDs From < BlinkerLEDs-1 > To < 0 > are SET to (100,240,0) & Blinking with delay of < BlinkerSpeed >
void LeftDim();											  // LEDs From < 0 > To < BlinkerLEDs > are Reset to (0,0,0)
void LeftLit();											  // LEDs From < 0 > To < MAX_LED - BlinkerLEDs > are set to Red color  (0,150,0)
void LeftFull();										  // LEDs From < 0 > To < MAX_LED - BlinkerLEDs > are Set Max Value Of Red Color (0,255,0)
void RightBlinker();                                      // LEDs From < BlinkerLEDs-1 > To < 0 > are SET to (100,240,0) & Blinking with delay of < BlinkerSpeed >
void RightDim(); 										  // LEDs From < MAX_LED - BlinkerLEDs > To < MAX_LED > are Reset to (0,0,0)
void RightLit(); 										  // LEDs From < BlinkerLEDs > To < MAX_LED  > are set to Red color  (0,150,0)
void RightFull(); 										  // LEDs From < BlinkerLEDs > To < MAX_LED > are Set Max Value Of Red Color (0,255,0)
void DualBlinker();                                       // LEDs From < 0 > To < BlinkerLEDs > AND LEDs From < MAX_LED - BlinkerLEDs > To < MAX_LED > are SET to (100,240,0) & Blinking with delay of < BlinkerSpeed >







#endif /* INC_WS2812_H_ */
