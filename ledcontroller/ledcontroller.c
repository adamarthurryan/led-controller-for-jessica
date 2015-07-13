/* Name: Simple PWM RGB LED
 * Author: Adam Brown
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#include "common.h"

void setupTimer();
void loop();

void hsvToRgb(float *r, float *g, float *b, float h, float s, float v);
void updateHsvColor();

void loadHsvPos(float *h, float *s, float *v);
void saveHsvPos(float h, float s, float v);


void delay(unsigned long);


//define the states for this program
enum STATE {
	SELECT,
	HOLD
};

enum CHANNEL {
	HUE,
	SAT,
	VAL
};

//the current state
enum STATE state;
enum CHANNEL channel;

#define RED 0
#define GREEN 1
#define BLUE 2

#define BUTTON_PIN 3
#define MSEC_BUTTON_DEBOUNCE 250

#define HUE_STEP 0.005
#define SAT_STEP 0.010
#define VAL_STEP 0.010
#define STEP_DELAY 20


//the current hue 
float huePos;
//the current saturation 
float satPos;
//the current color value (brightness)
float valPos;


//the red, green and blue pins
byte pins[3]={2,1,0};

//the length of the duty cycle (in ticks per 256)
volatile byte duty[3]={0,0,0};

//the number of ticks counted on the millisecond clock
volatile long millis=0;

byte EEMEM test;

int main(void)
{	
  //write enable port 0, 1, 2
  //read enable port 3, 4, 5 (implicit)
  DDRB = (1<<PB0) | (1<<PB1) | (1<<PB2);
  
  //disable pullups
  //???
  //MCUCR |= PUD;
  
  //configure the PWM and timer interrupts
  setupTimer();

	//set the initial state
	state=HOLD;
	channel=HUE;
	
	//if the button is held down, reset the color
	if (readPin(BUTTON_PIN)==false) {
		huePos=0.001;
		satPos=1;
		valPos=1;
		saveHsvPos(huePos, satPos, valPos);
		
		//wait until the button is released
		do {
			delay(MSEC_BUTTON_DEBOUNCE);
		} while (readPin(BUTTON_PIN)==false);
	}
	//otherwise load the saved color
	else {
		loadHsvPos(&huePos, &satPos, &valPos);
	}

	//set the initial color
	updateHsvColor();	

	while(true) {
		loop();
		//writePin(pins[0], true);
		//writePin(pins[1], true);
		//writePin(pins[2], false);
	}
	
	return 0;
}



void loop()
{
	//read the button value (false: pressed)
	boolean button=readPin(BUTTON_PIN);
	
	//calculate the state transitions
	//hold become select
	if (state==HOLD && button==false) {
		state=SELECT;
		delay(MSEC_BUTTON_DEBOUNCE);
		return;
	}
	//select becomes hold for the next channel
	else if (state==SELECT && button==true) {
		//save the current channel positions
		saveHsvPos(huePos, satPos, valPos);
		state=HOLD;
		//choose the next channel
		channel = HUE;
		/*
		switch (channel) {
			case HUE:
				channel=SAT;
				break;
			case SAT:
				channel=VAL;
				break;
			case VAL:
				channel=HUE;
		}
		*/
		
		delay(MSEC_BUTTON_DEBOUNCE);
	}

	//handle the select mode
	if (state==SELECT && channel==HUE) {
		//increment the hue
		huePos+=HUE_STEP;
		//mod the hue by 1
		if (huePos>1)
			huePos-=floor(huePos);
	
		updateHsvColor();
	}
	//handle the saturation select mode
	else if (state==SELECT && channel==SAT) {
		//increment the saturation position
		satPos+=SAT_STEP;
		//mod the saturation position by 1
		if (satPos>1)
			satPos-=floor(satPos);
		
		//set the color
		updateHsvColor();
	}
	//handle the value select mode
	else if (state==SELECT && channel==VAL) {
		//increment the saturation
		valPos+=VAL_STEP;
		//mod the saturation by 1
		if (valPos>1)
			valPos-=floor(valPos);
		

		//set the color
		updateHsvColor();
	}
	
	delay(STEP_DELAY);
}

//delay for the given number of millis
void delay(unsigned long msDelay) {
	unsigned long start = millis;
	
	while (millis - start < msDelay)
		;
}

//the LED pwm timer interrupt
byte count=0;
ISR(TIM0_COMPA_vect) {
	int i;
	for (i=0;i<3;i++) {
		if (count==0 && duty[i]>0)
			writePin(pins[i],true);
		else if (count>duty[i])
			writePin(pins[i],false);	
	}
		
	count+=4;	
}

ISR(TIM1_OVF_vect) {
	millis++;

	//temp:
/*	if (millis % 1000>500)
		writePin(1,true);
	else
		writePin(1,false);
*/
}

//setup the LED pwm timer
void setupTimer() {
	//disable interrupts
	cli();

	// Enable interrupt when timer 0 hits OCR0A
	TIMSK = _BV(OCIE0A) ;

	// Set CTC mode (Clear Timer on Compare Match) and no prescaler
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR0A = _BV(WGM01) ;   ///WGM01 (p. 82)
//	TCCR0B = _BV(CS00);  //CS00 (p. 83),
	TCCR0B = _BV(CS01); //prescaler factor 4

	// Set the compare register .
	// This is a 16-bit register?, so we have to do this with
	// interrupts disabled to be safe.
//	OCR0A = microsecondsToClockCycles(12);
		OCR0A = 128; // = 6 us
	
	//set timer prescale to 32
	//so timer rate is CK/32 = 250 kHz = 4 us (period) = 1024 us to overflow
	TCCR1 = _BV(CS12) | _BV(CS11);
	// Enable interrupt when timer 1 overflows
	TIMSK |= _BV(TOIE1);

	//enable interrupts
	sei();
}

//the float values are scaled by 32000 and saved as 16 bit integers
uint16_t EEMEM eeHuePos;
uint16_t EEMEM eeSatPos;
uint16_t EEMEM eeValPos;


//loads the current hue, sat and val pos from eeprom
void loadHsvPos(float *h, float *s, float *v) {
	*h = ((float)eeprom_read_word(&eeHuePos)) /32000.0;
	*s = ((float)eeprom_read_word(&eeSatPos)) /32000.0;
	*v = ((float)eeprom_read_word(&eeValPos)) /32000.0;
}

//saves the current hue, sat and val pos to eeprom
void saveHsvPos(float h, float s, float v) {
	eeprom_write_word(&eeHuePos, (uint16_t) (h*32000.0));
	eeprom_write_word(&eeSatPos, (uint16_t) (s*32000.0));
	eeprom_write_word(&eeValPos, (uint16_t) (v*32000.0));
}

void updateHsvColor() {
	float h=huePos;
	//scale the saturation position by 2 and reflect around 1
	float s=abs((satPos*2.0)-1.0);
	//scale the saturation position by 2 and reflect around 1
	float v=abs((valPos*2.0)-1.0);


	float r=0;
	float g=0;
	float b=0;

	//find the rgb values	
	hsvToRgb(&r, &g, &b, h, s, v);

	//set the duty cycle
  duty[RED]=r*255; 
	duty[GREEN]=g*255;
	duty[BLUE]=b*255;
}

//converts from the HSV color space to RGB
//all values are in the unit interval
void hsvToRgb(float *r, float *g, float *b, float h, float s, float v) {
	//if saturation is zero the tint is achromatic
	if (s==0) {
		*r = *g = *b =v;
		return;
	}
	
	// scale hue to be in sector 0 to 5
	h *= 6.0; 
	
	byte sector= floor(h);
	float f=h-sector;
	
	float p = v*(1-s);
	float q = v*(1-s*f);
	float t = v*(1-s*(1-f));
	
	switch (sector) {
		case 0:
			*r = v; *g = t; *b = p;
			break;
		case 1:
			*r = q; *g = v; *b = p;
			break;
		case 2:
			*r = p; *g = v; *b = t;
			break;
		case 3:
			*r = p; *g = q; *b = v;
			break;
		case 4:
			*r = t; *g = p; *b = v;
			break;
		case 5:	default:
			*r = v; *g = p; *b = q;
			break;
	}
}
