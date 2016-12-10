#include "I2C.h"
#include "Display.h"
#include "Screen_Image.h"
#include <stdio.h>
#include "stdbool.h"

/* Define the I/O Display pin control */
#define 	Display_Start_Reset			P3OUT &= ~BIT2
#define 	Display_Stop_Reset			P3OUT |= BIT2
#define     Display_Reset_Pin_Enable	P3DIR |= BIT2

volatile unsigned long disp_map[32] = {};		// display map, store 32*32 bit information, each long store 32 bits disp info, from bottom is the MSB, top is the LSB
								// [] is the collum number.
void Display_Initial(void){
	//WDTCTL = WDTPW | WDTHOLD;

	Display_Start_Reset;
	_delay_cycles(120000);
	Display_Stop_Reset;
	_delay_cycles(3000);
    // Initialize I2C peripheral as master
    I2C_Master_Display_Init(I2C_SOURCE_CLOCK_SMCLK,I2C_SOURCE_CLOCK_FREQ,I2C_CLOCK_DISPLAY_FREQ);
	_delay_cycles(240000);

	Display_Reset_Pin_Enable;                   //   don't forget to pull this this P3.2 pin

	Display_Stop_Reset;
	_delay_cycles(3000);
	Display_Start_Reset;
	_delay_cycles(120000);
	Display_Stop_Reset;

	_delay_cycles(3000);

	Set_Display_On_Off(0xAE);										// Display Off (0xAE/0xAF)
	Set_Display_Clock(0x80);										// Set Clock as 350 Frames/Sec
	Set_Multiplex_Ratio(0x1F);										// 1/16 Duty (0x0F~0x3F)
	Set_Display_Offset(0x00);										// Shift Mapping RAM Counter (0x00~0x3F)
	Set_Start_Line(0x00);											// Set Mapping RAM Display Start Line (0x00~0x3F)
	Set_Charge_Pump(0x14);											// Enable Built-in DC/DC Converter (0x10/0x14)
	Set_Addressing_Mode(0x01);										// Set Page Addressing Mode (0x00/0x01/0x02)
	Set_Segment_Remap(0xA1);										// Set SEG/Column Mapping (0xA0/0xA1)
	Set_Common_Remap(0xC8);											// Set COM/Row Scan Direction (0xC0/0xC8)
	Set_Common_Config(0x12);										// Set Sequential Configuration (0x02/0x12)
	Set_Contrast_Control(0xFF);										// Set SEG Output Current
	Set_Precharge_Period(0xF1);										// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	Set_VCOMH(0x20);												// Set VCOM Deselect Level
	Set_Entire_Display(0xA4);										// Disable Entire Display On (0xA4/0xA5)
	Set_Inverse_Display(0xA6);										// Disable Inverse Display On (0xA6/0xA7)

	Clear_RAM();													// Initialize Memory to all 0x00's

	Set_Display_On_Off(0xAF);										// Display On (0xAE/0xAF)
	Turn_Display_On();

//	Set_Display_On_Off(0xAE);										// Turn it off
}

void Set_Contrast_Control_Register(unsigned char mod){
	I2C_Write_Packet_To_Display(0x00,0x01,0x81);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,mod);
	_delay_cycles(3000);
	return;
}

void Clear_RAM(void){
    unsigned char i,j;

    for(i=0;i<0x80;i++){
    	Set_Page_Address(0x00);
    	Set_Column_Address(i);
    	  for(j=0;j<4;j++){
    		  I2C_Write_Packet_To_Display(0x40,0x01,0x00);
    		  _delay_cycles(500);
    	  }
		}
    return;
}

void Display_Picture(unsigned char pic_up[],unsigned char pic_down[]){
    unsigned char i,j;

    for(i=0;i<0x40;i++){
    	Set_Page_Address(0x00);
    	Set_Column_Address(i+32);
    	  for(j=0;j<2;j++){
    		 I2C_Write_Packet_To_Display(0x40,0x01,pic_up[(i*2) + j]);
    		 __delay_cycles(800);
    	  	 }
    	  for(j=0;j<2;j++){
    		 I2C_Write_Packet_To_Display(0x40,0x01,pic_down[(i*2) + j]);
    		 __delay_cycles(800);
    	  	 }
		}
    return;
}

void Set_Start_Column(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0x00+(d%32));
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,0x20+(d/32));
	_delay_cycles(3000);
}

void Set_Addressing_Mode(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0x20);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Page_Address(unsigned char page){
    page = (0xb0 | page);											// Set Page Start Address (B0-B7h)
    I2C_Write_Packet_To_Display(0x00,0x01,page);
    _delay_cycles(3000);
	return;
}

void Set_Column_Address(unsigned char column){
	I2C_Write_Packet_To_Display(0x00,0x01,( 0x10 | (column>>4)));
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,(0x0f & column));
	_delay_cycles(3000);
	return;
}

void Set_Start_Line(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0x40|d);
	_delay_cycles(3000);
}

void Set_Contrast_Control(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0x81);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Charge_Pump(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0x8D);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Segment_Remap(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Entire_Display(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Inverse_Display(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Multiplex_Ratio(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xA8);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Display_On_Off(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Start_Page(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,(0xB0|d));
	_delay_cycles(3000);
}

void Set_Common_Remap(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Display_Offset(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xD3);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Display_Clock(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xD5);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Precharge_Period(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xD9);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_Common_Config(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xDA);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_VCOMH(unsigned char d){
	I2C_Write_Packet_To_Display(0x00,0x01,0xDB);
	_delay_cycles(3000);
	I2C_Write_Packet_To_Display(0x00,0x01,d);
	_delay_cycles(3000);
}

void Set_NOP(){
	I2C_Write_Packet_To_Display(0x00,0x01,0xE3);
	_delay_cycles(3000);
}

void Turn_Display_On(){

	Set_Charge_Pump(0x14);											// Enable Built-in DC/DC Converter (0x10/0x14)

	Set_Display_On_Off(0xAF);										// Display On (0xAE/0xAF)
}

void Turn_Display_Off(){

	Set_Display_On_Off(0xAE);										// Display Off (0xAE/0xAF)

	Set_Charge_Pump(0x10);											// Disable Built-in DC/DC Converter (0x10/0x14)
}


//=== add by wenyi 07-28-2016 =====================================
/*
 * Display procedure:
 *
 * 1. clear the display map using clearDispMap() to clear the existing image(unless you just want to add images)
 * 2. use setDispMap(unsigned int *img, int len, int x, int y) to put image on display map, *img is the image file,
 *  len is the image width, x and y is the corrdinate in display map.
 *  the image file format is the bit map, image is on the top left of bit map, see Screen_Image.h for example. 16bit high at most
 * 3. use displayUpdate() function to update display map into actually display screen
 */

//=== if you want to display new things on screen, call this function first, and then add images ===
//=== if you just want to append images, no need to call this one ===
void clearDispMap(){		// clear the display map
	int i=0;
	for(i=0;i<32;i++){
		disp_map[i] = 0x00000000;		// set the display map to all 0
	}
}

//=== set certain img to specific (x,y) position, img is stored as int(16bit)*width
void setDispMap(unsigned int *img, int len, int x, int y){
	/* *img is the img address, len is the width of the img (img hight here wont exceed 16 bits)
		the img hight + x shouldn't exceed 32, otherwise the img display is not predictable
	 * */
	int i = 0;
	unsigned long img_tmp = 0;		// store the img temp info
	for(i=0;i<len;i++){
		img_tmp = (unsigned long)img[i] << y;		// set the img to the right position per collum
		disp_map[x+i] |= img_tmp;		// add img to the display map, keep the others
	}
}

//=== set certain img to specific (x,y) position, img is stored as long(32bit)*width
void setDispMap_32bit(unsigned long *img, int len, int x, int y){
	/* *img is the img address, len is the width of the img (img hight here wont exceed 16 bits)
		the img hight + x shouldn't exceed 32, otherwise the img display is not predictable
	 * */
	int i = 0;
	unsigned long img_tmp = 0;		// store the img temp info
	for(i=0;i<len;i++){
		img_tmp = img[i] << y;		// set the img to the right position per collum
		disp_map[x+i] |= img_tmp;		// add img to the display map, keep the others
	}
}

//=== clear the display
void displayClear(){
	clearDispMap();
	displayUpdate();
}

//=== display loading version 1
void displayLoading(int state){		// display loading gps screen, state from 1 to 7, decide where the loading block goes
	clearDispMap();
	setDispMap(CHAR_G_LARGE, CHAR_G_LARGE_LEN, 7, 5);
	setDispMap(CHAR_P_LARGE, CHAR_P_LARGE_LEN, 13, 5);
	setDispMap(CHAR_S_LARGE, CHAR_S_LARGE_LEN, 19, 5);		// add "GPS" to display map
	setDispMap(SYM_LOADING_BAR, SYM_LOADING_BAR_LEN, 7, 17);	// add loading bar
	setDispMap(SYM_LOADING_BLOCK, SYM_LOADING_BLOCK_LEN, 7+(state-1)*2, 18);	// add loading bar
	displayUpdate();
}

//=== display loading version 2
void displayLoadingV2(int state){		// display loading gps screen, state from 0 to 3, decide where the loading block goes
	clearDispMap();
	setDispMap(CHAR_G_LARGE, CHAR_G_LARGE_LEN, 7, 7);
	setDispMap(CHAR_P_LARGE, CHAR_P_LARGE_LEN, 13, 7);
	setDispMap(CHAR_S_LARGE, CHAR_S_LARGE_LEN, 19, 7);		// add "GPS" to display map
	setDispMap(SYM_LOADING_BAR, SYM_LOADING_BAR_LEN, 7, 18);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK, SYM_LOADING_TILT_BLOCK_LEN, 7+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK, SYM_LOADING_TILT_BLOCK_LEN, 11+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK, SYM_LOADING_TILT_BLOCK_LEN, 15+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK, SYM_LOADING_TILT_BLOCK_LEN, 19+state, 19);	// add loading bar
	int t=0;
	unsigned long tmp = 0;
	for(t=8;t<8+3;t++){					// adjust the sliding bar, extra right bar move back to left
		tmp = disp_map[t+16];
		disp_map[t+16] = 0;
		disp_map[t] |= tmp;
	}
	displayUpdate();
}

//=== display loading version 3
void displayLoadingV3(int state){		// display loading gps screen, state from 0 to 3, decide where the loading block goes
	clearDispMap();
	setDispMap(CHAR_G_LARGE, CHAR_G_LARGE_LEN, 7, 7);
	setDispMap(CHAR_P_LARGE, CHAR_P_LARGE_LEN, 13, 7);
	setDispMap(CHAR_S_LARGE, CHAR_S_LARGE_LEN, 19, 7);		// add "GPS" to display map
	setDispMap(SYM_LOADING_BAR_V3, SYM_LOADING_BAR_V3_LEN, 7, 18);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V3, SYM_LOADING_TILT_BLOCK_V3_LEN, 7+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V3, SYM_LOADING_TILT_BLOCK_V3_LEN, 11+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V3, SYM_LOADING_TILT_BLOCK_V3_LEN, 15+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V3, SYM_LOADING_TILT_BLOCK_V3_LEN, 19+state, 19);	// add loading bar
	int t=0;
	unsigned long tmp = 0;
	for(t=8;t<8+1;t++){					// adjust the sliding bar, extra right bar move back to left
		tmp = disp_map[t+16];
		disp_map[t+16] = 0;
		disp_map[t] |= tmp;
	}
	displayUpdate();
}

//=== display loading version 4
void displayLoadingV4(int state){		// display loading gps screen, state from 0 to 3, decide where the loading block goes
	clearDispMap();
	setDispMap(CHAR_G_LARGE, CHAR_G_LARGE_LEN, 7, 7);
	setDispMap(CHAR_P_LARGE, CHAR_P_LARGE_LEN, 13, 7);
	setDispMap(CHAR_S_LARGE, CHAR_S_LARGE_LEN, 19, 7);		// add "GPS" to display map
	setDispMap(SYM_LOADING_BAR_V4, SYM_LOADING_BAR_V4_LEN, 7, 18);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V4, SYM_LOADING_TILT_BLOCK_V4_LEN, 7+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V4, SYM_LOADING_TILT_BLOCK_V4_LEN, 11+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V4, SYM_LOADING_TILT_BLOCK_V4_LEN, 15+state, 19);	// add loading bar
	setDispMap(SYM_LOADING_TILT_BLOCK_V4, SYM_LOADING_TILT_BLOCK_V4_LEN, 19+state, 19);	// add loading bar
	int t=0;
	unsigned long tmp = 0;
	for(t=8;t<8+2;t++){					// adjust the sliding bar, extra right bar move back to left
		tmp = disp_map[t+16];
		disp_map[t+16] = 0;
		disp_map[t] |= tmp;
	}
	displayUpdate();
}

//=== display mph or km/h screen
void displaySpeed(int speed, char unit){		// unit is 'k' or 'm', stand for km/h or mph
	clearDispMap();
	if(unit == 'k'){				// stand for km/h
		setDispMap(CHAR_K, CHAR_K_LEN, 7, 19);
		setDispMap(CHAR_m, CHAR_m_LEN, 12, 20);
		setDispMap(SYM_SLASH, SYM_SLASH_LEN, 18, 19);
		setDispMap(CHAR_h, CHAR_h_LEN, 23, 19);		// add "km/h" to display map
	}else{
		setDispMap(CHAR_M, CHAR_M_LEN, 9, 19);
		setDispMap(CHAR_P, CHAR_P_LEN, 15, 19);
		setDispMap(CHAR_H, CHAR_H_LEN, 20, 19);		// add "MPH" to display map
	}

	if(speed>=0 && speed<10){
		setDispMap(numberMappingLarge(speed), LARGE_NUM_LEN, 14, 7);	// add 1 digit large number to mid position
	}else if(speed>=10 && speed<100){
		setDispMap(numberMappingLarge(speed/10), LARGE_NUM_LEN, 10, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge(speed%10), LARGE_NUM_LEN, 17, 7);	// add large number, 1's digit
	}else if(speed>=100 && speed<999){
		setDispMap(numberMappingLarge(speed/100), LARGE_NUM_LEN, 7, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge((speed/10)%10), LARGE_NUM_LEN, 14, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge(speed%10), LARGE_NUM_LEN, 21, 7);	// add large number, 1's digit
	}
	displayUpdate();					// update the display, display on the screen
}

//=== display time and direction
void displayTime(int hour, int min, int direction){
	// direction mapping, 1: north, 2: north east, 3: east, 4, south east, 5: south, 6: south west, 7: west, 8 north west, clockwise
	clearDispMap();
	setDispMap(numberMappingSmall(hour/10), SMALL_NUM_LEN, 6, 16);
	setDispMap(numberMappingSmall(hour%10), SMALL_NUM_LEN, 11, 16);		// display hour
	setDispMap(SYM_COLON, SYM_COLON_LEN, 16, 18);						// display ':'
	setDispMap(numberMappingSmall(min/10), SMALL_NUM_LEN, 18, 16);
	setDispMap(numberMappingSmall(min%10), SMALL_NUM_LEN, 23, 16);		// display minute
	switch(direction){
	case 1:												// north
		setDispMap(CHAR_n, CHAR_n_LEN, 14, 9);
		break;
	case 2:												// north east
		setDispMap(CHAR_n, CHAR_n_LEN, 12, 9);
		setDispMap(CHAR_e, CHAR_e_LEN, 17, 9);
		break;
	case 3:												// east
		setDispMap(CHAR_e, CHAR_e_LEN, 14, 9);
		break;
	case 4:												// south east
		setDispMap(CHAR_s, CHAR_s_LEN, 12, 9);
		setDispMap(CHAR_e, CHAR_e_LEN, 17, 9);
		break;
	case 5:												// south
		setDispMap(CHAR_s, CHAR_s_LEN, 14, 9);
		break;
	case 6:												// south west
		setDispMap(CHAR_s, CHAR_s_LEN, 12, 9);
		setDispMap(CHAR_w, CHAR_w_LEN, 17, 9);
		break;
	case 7:												// west
		setDispMap(CHAR_w, CHAR_w_LEN, 14, 9);
		break;
	case 8: default:									// north west (northwestern :D)
		setDispMap(CHAR_n, CHAR_n_LEN, 12, 9);
		setDispMap(CHAR_w, CHAR_w_LEN, 17, 9);
		break;
	}
	displayUpdate();					// update the display, display on the screen
}

//=== display vertical drop
void displayVerticalDrop(long int drop, char unit){		// unit is 'm' or 'f', stand for m or feet
	clearDispMap();
	if(unit == 'm'){
		setDispMap(CHAR_M, CHAR_M_LEN, 14, 18);		// display 'm'
	}else{
		setDispMap(CHAR_F, CHAR_F_LEN, 13, 18);		// display 'ft'
		setDispMap(CHAR_T, CHAR_T_LEN, 17, 18);
	}
	int digit_1, digit_2, digit_3, digit_4, digit_5;	// 1's digit to 10000's digit respectively
	digit_1 = (int)(drop%10);
	digit_2 = (int)((drop/10)%10);
	digit_3 = (int)((drop/100)%10);
	digit_4 = (int)((drop/1000)%10);
	digit_5 = (int)((drop/10000)%10);
	if(drop>=0 && drop<10){
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 14, 9);
	}else if(drop>=10 && drop<100){
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 12, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 17, 9);
	}else if(drop>=100 && drop<1000){
		setDispMap(numberMappingSmall(digit_3), SMALL_NUM_LEN, 9, 9);
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 14, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 19, 9);
	}else if(drop>=1000 && drop<10000){
		setDispMap(numberMappingSmall(digit_4), SMALL_NUM_LEN, 7, 9);
		setDispMap(numberMappingSmall(digit_3), SMALL_NUM_LEN, 12, 9);
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 17, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 22, 9);
	}else if(drop>=10000 && drop<100000){
		setDispMap(numberMappingSmall(digit_5), SMALL_NUM_LEN, 4, 9);
		setDispMap(numberMappingSmall(digit_4), SMALL_NUM_LEN, 9, 9);
		setDispMap(numberMappingSmall(digit_3), SMALL_NUM_LEN, 14, 9);
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 19, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 24, 9);
	}
	displayUpdate();					// update the display, display on the screen
}

//=== display calorie
void displayCal(int cal){
	clearDispMap();
	setDispMap(CHAR_C, CHAR_C_LEN, 10, 18);
	setDispMap(CHAR_a, CHAR_a_LEN, 14, 20);
	setDispMap(CHAR_l, CHAR_l_LEN, 19, 18);
	int digit_1, digit_2, digit_3, digit_4;	// 1's digit to 1000's digit respectively
	digit_1 = cal%10;
	digit_2 = (cal/10)%10;
	digit_3 = (cal/100)%10;
	digit_4 = (cal/1000)%10;
	if(cal>=0 && cal<10){
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 14, 9);
	}else if(cal>=10 && cal<100){
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 11, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 16, 9);
	}else if(cal>=100 && cal<1000){
		setDispMap(numberMappingSmall(digit_3), SMALL_NUM_LEN, 9, 9);
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 14, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 19, 9);
	}else if(cal>=1000 && cal<10000){
		setDispMap(numberMappingSmall(digit_4), SMALL_NUM_LEN, 6, 9);
		setDispMap(numberMappingSmall(digit_3), SMALL_NUM_LEN, 11, 9);
		setDispMap(numberMappingSmall(digit_2), SMALL_NUM_LEN, 16, 9);
		setDispMap(numberMappingSmall(digit_1), SMALL_NUM_LEN, 21, 9);
	}
	displayUpdate();					// update the display, display on the screen
}

//=== display direction, for navigation

void displayNavigationArrow(int direction, int distance, int arrow_on, char unit){	// 'm' stand for meter, 'f' for feet, 'i' for mile, arrow_on control whether display arrow or not, used in arrow blink
	clearDispMap();
	setDispMap_32bit(PIC_DIRECTION_CIRCLE, PIC_DIRECTION_CIRCLE_LEN, 4, 4);
	if(arrow_on == 1){
		switch(direction){
		case 1:				// front
			setDispMap(SYM_ARROW_FRONT, SYM_ARROW_FRONT_LEN, 14, 5);
			break;
		case 2:
			setDispMap(SYM_ARROW_RIGHT_FRONT, SYM_ARROW_RIGHT_FRONT_LEN, 21, 8);
			break;
		case 3:
			setDispMap(SYM_ARROW_RIGHT, SYM_ARROW_RIGHT_LEN, 25, 14);
			break;
		case 4:
			setDispMap(SYM_ARROW_RIGHT_REAR, SYM_ARROW_RIGHT_REAR_LEN, 21, 22);
			break;
		case 5:
			setDispMap(SYM_ARROW_REAR, SYM_ARROW_REAR_LEN, 14, 25);
			break;
		case 6:
			setDispMap(SYM_ARROW_LEFT_REAR, SYM_ARROW_LEFT_REAR_LEN, 9, 22);
			break;
		case 7:
			setDispMap(SYM_ARROW_LEFT, SYM_ARROW_LEFT_LEN, 5, 14);
			break;
		case 8:
			setDispMap(SYM_ARROW_LEFT_FRONT, SYM_ARROW_LEFT_FRONT_LEN, 9, 8);
			break;
		}
	}

	switch(unit){
	case 'm':
		setDispMap(CHAR_M, CHAR_M_LEN, 14, 18);		// display 'm'
		break;
	case 'f':
		setDispMap(CHAR_F, CHAR_F_LEN, 13, 18);		// display 'ft'
		setDispMap(CHAR_T, CHAR_T_LEN, 17, 18);
		break;
	case 'i': default:
		setDispMap(CHAR_m_SMALL, CHAR_m_SMALL_LEN, 13, 19);
		setDispMap(CHAR_i_SMALL, CHAR_i_SMALL_LEN, 19, 18);		// display 'mi'
		break;
	}

	if(unit == 'm' || unit == 'f'){		// feet and meter
		int digit_1, digit_2, digit_3, digit_4;	// 1's digit to 1000's digit respectively
		digit_1 = distance%10;
		digit_2 = (distance/10)%10;
		digit_3 = (distance/100)%10;
		digit_4 = (distance/1000)%10;
		if(distance>=0 && distance<10){
			setDispMap(numberMappingTINY(digit_1), TINY_NUM_LEN, 15, 12);
		}else if(distance>=10 && distance<100){
			setDispMap(numberMappingTINY(digit_2), TINY_NUM_LEN, 13, 12);
			setDispMap(numberMappingTINY(digit_1), TINY_NUM_LEN, 17, 12);
		}else if(distance>=100 && distance<1000){
			setDispMap(numberMappingTINY(digit_3), TINY_NUM_LEN, 11, 12);
			setDispMap(numberMappingTINY(digit_2), TINY_NUM_LEN, 15, 12);
			setDispMap(numberMappingTINY(digit_1), TINY_NUM_LEN, 19, 12);
		}else if(distance>=1000 && distance<10000){
			setDispMap(numberMappingTINY(digit_4), TINY_NUM_LEN, 9, 12);
			setDispMap(numberMappingTINY(digit_3), TINY_NUM_LEN, 13, 12);
			setDispMap(numberMappingTINY(digit_2), TINY_NUM_LEN, 17, 12);
			setDispMap(numberMappingTINY(digit_1), TINY_NUM_LEN, 21, 12);
		}
	}else{			// mile
		int digit_1, digit_2, digit_3;	// 1's digit to 1000's digit respectively
		digit_1 = distance%10;
		digit_2 = (distance/10)%10;
		digit_3 = (distance/100)%10;
		setDispMap(numberMappingTINY(digit_3), TINY_NUM_LEN, 10, 12);
		setDispMap(SYM_DOT, SYM_DOT_LEN, 14, 16);
		setDispMap(numberMappingTINY(digit_2), TINY_NUM_LEN, 16, 12);
		setDispMap(numberMappingTINY(digit_1), TINY_NUM_LEN, 20, 12);
	}

	displayUpdate();					// update the display, display on the screen
}




//=== display max speed
void displayMaxSpeed(int mph){
	clearDispMap();
	setDispMap(CHAR_M_LARGE, CHAR_M_LARGE_LEN, 6, 18);
	setDispMap(CHAR_A_LARGE, CHAR_A_LARGE_LEN, 13, 18);
	setDispMap(CHAR_X_LARGE, CHAR_X_LARGE_LEN, 20, 18);		// add "max" to display map
	if(mph>=0 && mph<10){
		setDispMap(numberMappingLarge(mph), LARGE_NUM_LEN, 13, 7);	// add 1 digit large number to mid position
	}else if(mph>=10 && mph<100){
		setDispMap(numberMappingLarge(mph/10), LARGE_NUM_LEN, 9, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge(mph%10), LARGE_NUM_LEN, 16, 7);	// add large number, ten's digit
	}else if(mph>=100 && mph<999){
		setDispMap(numberMappingLarge(mph/100), LARGE_NUM_LEN, 7, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge((mph/10)%10), LARGE_NUM_LEN, 13, 7);	// add large number, ten's digit
		setDispMap(numberMappingLarge(mph%10), LARGE_NUM_LEN, 19, 7);	// add large number, ten's digit
	}
	displayUpdate();					// update the display, display on the screen
}

//=== display destination picture
void displayDestinationPic(){
	clearDispMap();
	setDispMap_32bit(PIC_DESTINATION, PIC_DESTINATION_LEN, 7, 5);
	displayUpdate();
}

//=== display finding friend picture
void displayFindFriend(){
	clearDispMap();
	setDispMap_32bit(PIC_FIND_FRIEND, PIC_FIND_FRIEND_LEN, 4, 4);
	displayUpdate();
}

//=== display phone picture for incoming call
void displayPhoneCall(){
	clearDispMap();
	setDispMap_32bit(PIC_PHONE, PIC_PHONE_LEN, 6, 7);
	displayUpdate();
}

//=== display message picture
void displayMessage(){
	clearDispMap();
	setDispMap_32bit(PIC_MESSAGE, PIC_MESSAGE_LEN, 5, 6);
	displayUpdate();
}

//=== display email picture
void displayEmail(){
	clearDispMap();
	setDispMap_32bit(PIC_EMAIL, PIC_EMAIL_LEN, 4, 7);
	displayUpdate();
}

//=== display ble reset pic
void displayBLEReset(){
	clearDispMap();
	setDispMap_32bit(PIC_BT_RESET, PIC_BT_RESET_LEN, 5, 2);
	displayUpdate();
}

//=== display low battery
void displayLowBattery(){
	clearDispMap();
	setDispMap_32bit(PIC_BATTERY_LOW, PIC_BATTERY_LOW_LEN, 5, 10);
	displayUpdate();
}

//=== display logo icon when shutdown
void displayLogo(){
	clearDispMap();
	setDispMap_32bit(PIC_GOGGLEPAL_LOGO, PIC_GOGGLEPAL_LOGO_LEN, 4, 4);
	displayUpdate();
}

//=== display startup animation
void displayStartupAnimation(){
	int i=0;
	int x=4,y=4;					// x and y offset
	for(i=0;i<2;i++){				// flash logo dot
		clearDispMap();
		displayUpdate();
		__delay_cycles(2400000);
		setDispMap(SYM_LOGO_DOT, SYM_LOGO_DOT_LEN, 14+x, 14+y);
		displayUpdate();
		__delay_cycles(2400000);
	}
	// animation begin
	for(i=0;i<6;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 9+i+x, 0+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 15+i+x, 1+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<4;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 18+i+x, 2+i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 22+x, 6+i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<6;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 23+x, 9+i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 22+x, 15+i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<4;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 21-i+x, 18+i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 17-i+x, 22+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<6;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 14-i+x, 23+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 8-i+x, 22+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<4;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 5-i+x, 21-i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 1+x, 17-i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<6;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 0+x, 14-i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 1+x, 8-i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<4;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 2+i+x, 5-i+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}
	for(i=0;i<3;i++){
		setDispMap(SYM_DOT, SYM_DOT_LEN, 6+i+x, 1+y);
		displayUpdate();
		//delay_10000_cycles_n_times(3);
	}

}

//=== need to call this function everytime you want to display new image ===
//=== it update the entire screen by loading the display map info 		 ===
void displayUpdate(){
	int i=0;
	for(i=0;i<32;i++){
		Set_Page_Address(0x00);
		Set_Column_Address(36+i);
		display1Byte((unsigned char)(disp_map[i] & 0xff));
		display1Byte((unsigned char)(disp_map[i] >> 8) & 0xff);
		display1Byte((unsigned char)(disp_map[i] >> 16) & 0xff);
		display1Byte((unsigned char)(disp_map[i] >> 24) & 0xff);

	}
}

//=== just for display 1 byte(8 bit) on display, need to call set page and address function before using this one
void display1Byte(unsigned char data){
	I2C_Write_Packet_To_Display(0x40,0x01,data);
	__delay_cycles(400);           //8MhzÃƒï¿½Ãƒâ€šÃƒÆ’Ã‚Â¿Ã‚Â¸ÃƒÂ¶Ãƒï¿½Ãƒâ€�ÃƒÅ Ã‚Â¾Ãƒâ€˜Ãƒâ€œÃ‚Â³Ãƒâ„¢Ã‚Â²Ã‚Â»Ãƒâ€žÃƒÅ“Ã‚ÂµÃƒï¿½Ãƒâ€œÃƒÅ¡300Ã‚Â£Ã‚Â¬Ã‚Â·ÃƒÂ±Ãƒâ€�ÃƒÂ²Ãƒï¿½Ã‚Â¢Ã‚Â±Ãƒâ‚¬
}

//=== just for delay n * 10000 cycles
void delay_10000_cycles_n_times(int n){
	int i=0;
	for(i=0;i<n;i++){
		__delay_cycles(10000);
	}
}

unsigned int* numberMappingTINY(int digit){          // mapping the numbers to picture array
	switch(digit){
	case 0: return NUM_0_TINY;
	case 1: return NUM_1_TINY;
	case 2: return NUM_2_TINY;
	case 3: return NUM_3_TINY;
	case 4: return NUM_4_TINY;
	case 5: return NUM_5_TINY;
	case 6: return NUM_6_TINY;
	case 7: return NUM_7_TINY;
	case 8: return NUM_8_TINY;
	case 9: return NUM_9_TINY;
	}
	return NUM_8_TINY;
}

unsigned int* numberMappingSmall(int digit){          // mapping the numbers to picture array
	switch(digit){
	case 0: return NUM_0_SMALL;
	case 1: return NUM_1_SMALL;
	case 2: return NUM_2_SMALL;
	case 3: return NUM_3_SMALL;
	case 4: return NUM_4_SMALL;
	case 5: return NUM_5_SMALL;
	case 6: return NUM_6_SMALL;
	case 7: return NUM_7_SMALL;
	case 8: return NUM_8_SMALL;
	case 9: return NUM_9_SMALL;
	}
	return NUM_8_SMALL;
}

unsigned int* numberMappingLarge(int digit){          // mapping the numbers to picture array
	switch(digit){
	case 0: return NUM_0_LARGE;
	case 1: return NUM_1_LARGE;
	case 2: return NUM_2_LARGE;
	case 3: return NUM_3_LARGE;
	case 4: return NUM_4_LARGE;
	case 5: return NUM_5_LARGE;
	case 6: return NUM_6_LARGE;
	case 7: return NUM_7_LARGE;
	case 8: return NUM_8_LARGE;
	case 9: return NUM_9_LARGE;
	}
	return NUM_8_LARGE;
}

//======== old version functions ============================================

/*void displayNavigationArrow(int direction){
	// direction mapping, 1: front, 2: right front, 3: right, 4, right rear, 5: rear, 6: left rear, 7: left, 8 left front, clockwise
	clearDispMap();
	switch(direction){
	case 1:				// front
		setDispMap(SYM_ARROW_FRONT, SYM_ARROW_FRONT_LEN, 9, 7);
		break;
	case 2:
		setDispMap(SYM_ARROW_RIGHT_FRONT, SYM_ARROW_RIGHT_FRONT_LEN, 7, 10);
		break;
	case 3:
		setDispMap(SYM_ARROW_RIGHT, SYM_ARROW_RIGHT_LEN, 8, 9);
		break;
	case 4:
		setDispMap(SYM_ARROW_RIGHT_REAR, SYM_ARROW_RIGHT_REAR_LEN, 7, 7);
		break;
	case 5:
		setDispMap(SYM_ARROW_REAR, SYM_ARROW_REAR_LEN, 9, 7);
		break;
	case 6:
		setDispMap(SYM_ARROW_LEFT_REAR, SYM_ARROW_LEFT_REAR_LEN, 10, 7);
		break;
	case 7:
		setDispMap(SYM_ARROW_LEFT, SYM_ARROW_LEFT_LEN, 8, 9);
		break;
	case 8:
		setDispMap(SYM_ARROW_LEFT_FRONT, SYM_ARROW_LEFT_FRONT_LEN, 10, 10);
		break;
	}
	displayUpdate();
}*/

//======================= for testing the bosch 4 axis calibration
//=== display calorie
void displayCalibration(int calib_state, int roll, int pitch){
	clearDispMap();
	int off_x=roll/8;
	int off_y=pitch/8;		// resolution is 8 degree/pixel
	int x=off_x+15;
	int y=off_y+15;			// origin point is 15,15
	x=x>0?x:0;
	x=x<32?x:31;		// set x range 0 to 31, dont exceed boundary
	y=y>0?y:0;
	y=y<32?y:0;			// set y range 0 to 31
	switch(calib_state){
	case 1:			// do forward pitch
		setDispMap(SYM_DOT, SYM_DOT_LEN, 15, 28);
		setDispMap(SYM_DOT, SYM_DOT_LEN, 16, 28);	// set destination marker
		setDispMap(SYM_DOT, SYM_DOT_LEN, x, y);	// set current point based on roll and pitch
		break;
	case 2:			// do rear pitch
		setDispMap(SYM_DOT, SYM_DOT_LEN, 15, 4);
		setDispMap(SYM_DOT, SYM_DOT_LEN, 16, 4);	// set destination marker
		setDispMap(SYM_DOT, SYM_DOT_LEN, x, y);	// set current point based on roll and pitch
		break;
	case 3:			// do left roll
		setDispMap(SYM_DOT, SYM_DOT_LEN, 4, 15);
		setDispMap(SYM_DOT, SYM_DOT_LEN, 4, 16);	// set destination marker
		setDispMap(SYM_DOT, SYM_DOT_LEN, x, y);	// set current point based on roll and pitch
		break;
	case 4:			// do right roll
		setDispMap(SYM_DOT, SYM_DOT_LEN, 28, 15);
		setDispMap(SYM_DOT, SYM_DOT_LEN, 28, 16);	// set destination marker
		setDispMap(SYM_DOT, SYM_DOT_LEN, x, y);	// set current point based on roll and pitch
		break;
	}
	displayUpdate();					// update the display, display on the screen
}
