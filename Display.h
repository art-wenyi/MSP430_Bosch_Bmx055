
#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "I2C.h"
#include "msp430f5659.h"

/* Define the various Display screens */
#define					GPS_Screen						0
#define 				MaxSpeed_Screen					1
#define					Time_Screen						2
#define					Vertical_Drop_Screen			3
#define 				Calorie_Screen					4
#define 				Navigation_Screen				5
#define					Loading_Screen					6
#define					PhoneCall_Screen				7
#define					Message_Screen					8
#define					Email_Screen					9
#define 				BLE_Reset_Screen				10
#define					Low_Battery_Screen				11
#define 				Logo_Screen						12

/* Define the various Navigation screens */
#define					Arrow_N_Screen					1
#define					Arrow_NE_Screen					2
#define					Arrow_E_Screen					3
#define					Arrow_SE_Screen					4
#define					Arrow_S_Screen					5
#define					Arrow_SW_Screen					6
#define					Arrow_W_Screen					7
#define					Arrow_NW_Screen					8
#define 				Destination_Screen				9

void Display_Initial(void);
void Display_Write_Instruction(unsigned char command);
void Display_Write_Data(unsigned char data);
void Set_Contrast_Control_Register(unsigned char mod);
void Display_Picture(unsigned char pic_up[],unsigned char pic_down[]);
extern void Set_Start_Column(unsigned char d);
void Set_Addressing_Mode(unsigned char d);
void Set_Page_Address(unsigned char page);
void Set_Column_Address(unsigned char column);
void Set_Start_Line(unsigned char d);
void Set_Contrast_Control(unsigned char d);
void Set_Charge_Pump(unsigned char d);
void Set_Segment_Remap(unsigned char d);
void Set_Entire_Display(unsigned char d);
void Set_Inverse_Display(unsigned char d);
void Set_Multiplex_Ratio(unsigned char d);
void Set_Display_On_Off(unsigned char d);
void Set_Start_Page(unsigned char d);
void Set_Common_Remap(unsigned char d);
void Set_Display_Offset(unsigned char d);
void Set_Display_Clock(unsigned char d);
void Set_Precharge_Period(unsigned char d);
void Set_Common_Config(unsigned char d);
void Set_VCOMH(unsigned char d);
void Set_NOP();
void Fill_RAM(unsigned char Data);
void Clear_RAM(void);
void Turn_Display_On(void);
void Turn_Display_Off(void);

//=== below are functoins add by wenyi, 07/28/2016 =======================================
//=== use for the Gogglepal 1.1 display project ===================================

void clearDispMap();
void setDispMap(unsigned int *img, int len, int x, int y);
void setDispMap_32bit(unsigned long *img, int len, int x, int y);
void displayClear();
void displayLoading(int state);
void displayLoadingV2(int state);
void displayLoadingV3(int state);
void displayLoadingV4(int state);
void displaySpeed(int speed, char unit);
void displayTime(int hour, int min, int direction); //direction mapping 1: north, 2: north east, 3: east, 4, south east, 5: south, 6: south west, 7: west, 8 north west, closkwise
void displayVerticalDrop(long int drop, char unit);
void displayCal(int cal);
void displayNavigationArrow(int direction, int distance, int arrow_on, char unit);
void displayMaxSpeed(int mph);
void displayDestinationPic();
void displayFindFriend();
void displayPhoneCall();
void displayMessage();
void displayEmail();
void displayBLEReset();
void displayLowBattery();
void displayLogo();
void displayStartupAnimation();

void displayUpdate();
void delay_10000_cycles_n_times(int n);
unsigned int* numberMappingTINY(int digit);
unsigned int* numberMappingSmall(int digit);
unsigned int* numberMappingLarge(int digit);

void display1Byte(unsigned char data);
#endif /* DISPLAY_H_ */
