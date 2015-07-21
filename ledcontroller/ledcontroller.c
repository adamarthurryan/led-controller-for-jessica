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

//#define DEBUG_HUE

void setupTimer();
void loop();

void hsvToRgb(float *r, float *g, float *b, float h, float s, float v);
void updateColor(float h);
void updateBlack();

void loadABColors(float *hueA, float *hueB);
void saveABColors(float hueA, float hueB);


void delay(unsigned long);

void resetMillis();


//define the states for this program
enum STATE {
	ANIMATE,
	SELECT,
	HOLD,
	START
};

enum CHANNEL {
	A = 0,
	B = 1
};

//the current state
enum STATE state;
enum CHANNEL channel;

unsigned long msHoldStart = 0;
unsigned long msAnimateStart = 0;

#define MSEC_HOLD_DELAY 2000
#define MSEC_ANIMATE_CYCLE 24000
#define MSEC_SOLID 4000
#define MSEC_FADE 8000

#define MSEC_RESET_THRESHOLD 50
#define RESET_COUNT 70000

#define RED 0
#define GREEN 1
#define BLUE 2

#define BUTTON_PIN 3
#define MSEC_BUTTON_DEBOUNCE 250

#define HUE_STEP 0.001
#define STEP_DELAY 20


//the current hue for channels A and B
float hue[] = {0.0, 0.0};


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
  DDRB = (1<<PB0) | (1<<PB1) | (1<<PB2);
  
  //read enable port 3 
  DDRB |= (0<<PB3);
  
  //enable port 3 pullup
  PORTB |= (1<<PB3);

  //configure the PWM and timer interrupts
  setupTimer();

	//set the initial state
	state=ANIMATE;
	msAnimateStart = millis;
	channel=A;
	
	//if the button is held down, reset the color
	if (readPin(BUTTON_PIN)==false) {
		hue[A]=0.001;
		hue[B]=0.501;
		saveABColors(hue[A], hue[B]);
		
		//wait until the button is released
		do {
			delay(MSEC_BUTTON_DEBOUNCE);
		} while (readPin(BUTTON_PIN)==false);
	}
	//otherwise load the saved color
	else {
		loadABColors(&hue[A], &hue[B]);
	}

	//set the initial color
	//updateColor(hue[A]);	

	//input handling loop
	while(true) {
		loop();
	}
	
	return 0;
}



void loop()
{
	//read the button value (false: pressed)
	boolean button=readPin(BUTTON_PIN);
	
	//calculate the state transitions
	//hold or animate become select when the button is pressed
	if ((state==HOLD || state==ANIMATE) && button==false) {
		state=SELECT;
		updateColor(hue[channel]);
		delay(MSEC_BUTTON_DEBOUNCE);
		return;
	}
	//select becomes hold for the next channel
	else if (state==SELECT && button==true) {
		//save the current channel positions
		saveABColors(hue[A], hue[B]);
		state=HOLD;
		
		msHoldStart = millis;
		
		//choose the next channel
		switch (channel) {
			case A:
				channel=B;
				break;
			case B:
				channel=A;
				break;
		}
		
		delay(MSEC_BUTTON_DEBOUNCE);
	}

	//handle the select mode
	else if (state==SELECT && button==false) {
		//increment the hue
		hue[channel]+=HUE_STEP;
		//mod the hue by 1
		if (hue[channel]>1)
			hue[channel]-=floor(hue[channel]);
	
		updateColor(hue[channel]);
	}
	
	//hold becomes animate after a delay, or when first starting
	else if (state==START || (state==HOLD && (millis-msHoldStart > MSEC_HOLD_DELAY))) {
		
		//flash the colors to indicate animate start
		updateBlack();
		delay(500);
		updateColor(hue[A]);
		delay(500);
		updateBlack();
		delay(500);
		updateColor(hue[B]);
		delay(500);
		updateBlack();
		delay(500);
		
		
		state = ANIMATE;
		msAnimateStart = millis;
	}
	
	//handle animation
	else if (state==ANIMATE) {				
		//get the number of ms elapsed since the start of the current animation cycle
		unsigned long msCycleElapsed = (millis - msAnimateStart) % MSEC_ANIMATE_CYCLE;
		int cycleNumber = (millis-msAnimateStart)/MSEC_ANIMATE_CYCLE;
		
		//every so often, we'll reset the milliseconds counter
		if (msCycleElapsed < MSEC_RESET_THRESHOLD && cycleNumber > RESET_COUNT) {
			resetMillis();
		}
		
		float hueStart;
		float hueEnd;
		
		//first the animation fades from A to B and then from B to A
		//each of these halves takes half of the animation cycle
		if (msCycleElapsed < MSEC_ANIMATE_CYCLE/2) {
			hueStart = hue[A];
			hueEnd = hue[B];
		}
		else {
			hueStart = hue[B];
			hueEnd = hue[A];
		}
		
		
#ifdef DEBUG_HUE
	hueEnd=0.4;
	hueStart=0.91;
#endif
		
		//get the ms elapsed since the start of the current half of the animation cycle
		msCycleElapsed = msCycleElapsed % (MSEC_ANIMATE_CYCLE/2);
		
		//if we are still in the solid part of the cycle, just show the starting color
		if (msCycleElapsed <= MSEC_SOLID)
			updateColor(hueStart);
			
		//otherwise interpolate between the starting and ending color
		else {
			float interpolate = (float) (msCycleElapsed-MSEC_SOLID)/MSEC_FADE;
			
			
			//we want to interpolate along the shortest distance
			//so, if the start and end colors are more than half the number wheel distant
			if (hueStart - hueEnd > 0.5)
				hueEnd += 1.0;
			else if (hueEnd - hueStart > 0.5)
				hueStart += 1.0;
			
			
			
			float hue = hueEnd*interpolate + hueStart*(1.0-interpolate);
			
			if (hue >= 1.0)
				hue -= 1.0;
			
			updateColor(hue);
		}
	}
		
	
	delay(STEP_DELAY);
}

//reset the millisecond counter and notify attached devices
void resetMillis() {
	//!!! not currently implemented
	msHoldStart -= millis;
	msAnimateStart -= millis;
	millis = 0;
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
uint16_t EEMEM eeHueA;
uint16_t EEMEM eeHueB;


//loads the current hue, sat and val pos from eeprom
void loadABColors(float *ha, float *hb) {
	*ha = ((float)eeprom_read_word(&eeHueA)) /32000.0;
	*hb = ((float)eeprom_read_word(&eeHueB)) /32000.0;
}

//saves the current hue, sat and val pos to eeprom
void saveABColors(float ha, float hb) {
	eeprom_write_word(&eeHueA, (uint16_t) (ha*32000.0));
	eeprom_write_word(&eeHueB, (uint16_t) (hb*32000.0));
}


//sets the color to black
void updateBlack() {
	for (int i=0; i<3; i++)
		duty[i]=0;
}

//sets the colour to the given hue
void updateColor(float h) {

	//scale the saturation position by 2 and reflect around 1
	//s=abs((s*2.0)-1.0);
	//scale the saturation position by 2 and reflect around 1
	//v=abs((v*2.0)-1.0);

	float s = 1;
	float v = 1;

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
