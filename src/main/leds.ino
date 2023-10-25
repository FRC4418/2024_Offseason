/*
A simple program that controls an addressable led strip based on i2c commands 
the program exapects a single byte over i2c representing an integer value 
on startup the strip will perform a small startup animation
Need change over to CHSV from CRGB to make fading out easier and more color accurate 
*/

/*to deploy this code, copy and paste it into the arduino IDE and install the fastLED library
after compiling once, turn on the robot and plug in the arduino to your computer
select the Atmel atmega328p Xplained mini board and select the correct COM port and deploy
the LEDs should turn on after doing so.
NOTE - Ensure the correct data pins are defined
*/ 
#include <Wire.h>

#include <FastLED.h>

#define NUM_LEDS 120
#define DATA_PIN 6
#define DATA_PIN_2 7
#define BRIGHTNESS 30
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100
#define FORWARD_COLOR_RED_ALLIANCE Red
#define FORWARD_COLOR_BLUE_ALLIANCE Blue

CRGB leds[NUM_LEDS];

#define I2C_ADDRESS 0x44
volatile int new_data = 0;
volatile int value = 0;
volatile int value_register = 0;


void setup() {
	delay(1000); // power-up safety delay
	FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
	FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
	startupEffect();
	
	Wire.begin(I2C_ADDRESS);
	Wire.onReceive(i2cReceiveEvent);
	
	FastLED.setBrightness(BRIGHTNESS);
}


void loop() {
	if (new_data) {
		new_data = 0;

		switch (value) {
			default:
				for (int i = 0; i < NUM_LEDS; i++) {
					leds[i] = CRGB::Black;
				 }
				break;
				
			case 5:
				startupEffect();
				break;
				
			case 4:
				for (int i = 0; i < NUM_LEDS; i++) {
					leds[i] = CRGB::Blue;
				 }
				break;
				 
			case 3:
				for (int i = 0; i < NUM_LEDS; i++) {
					leds[i] = CRGB::Green;
				 }
				 break;

			case 2:
				for (int i = NUM_LEDS/2; i < NUM_LEDS; i++) {
					leds[i] = CRGB::FORWARD_COLOR_RED_ALLIANCE;
				}
				for (int i = 0; i < NUM_LEDS/2; i++) {
					leds[i] = CRGB::Black;
				}
				break;

			case 1:
				for (int i = NUM_LEDS/2; i < NUM_LEDS; i++) {
					leds[i] = CRGB::Black;
				}
				for (int i = 0; i < NUM_LEDS/2; i++) {
					leds[i] = CRGB::FORWARD_COLOR_RED_ALLIANCE;
				}
				break;
		}
	}
	FastLED.show();
}


void i2cReceiveEvent(int bytesReceived) {  //The first byte is the register and the rest of the bytes are data
	value_register = (int) Wire.read();
	value = (int) Wire.read(); 
	if (bytesReceived > 2) {   //Throw away all the rest of the data past the first 2 bytes
		for (uint8_t a = 2; a < bytesReceived; a++) {  
			Wire.read();
		}
	}
	new_data = 1;
}


void startupEffect() {
	Wire.end();
	meteorRainHalfFill(0xff, 0xff, 0xff, 8, 100, true, .5, 1);
	for (int j = 255; j > 0; j--) {
		for (int i = 0; i < NUM_LEDS; i++ ) {
			leds[i].r = j;
			leds[i].g = j;
			leds[i].b = j;
		}
		FastLED.show();
		delay(2);
	}
	Wire.begin(I2C_ADDRESS);
	Wire.onReceive(i2cReceiveEvent);
}


void meteorRainFill(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay, int FillDelay) {  
	setAll(0, 0, 0);
 
	for (int i = 0; i < NUM_LEDS; i++) {
	 
	 
		// fade brightness all LEDs one step
		for (int j = 0; j<NUM_LEDS; j++) {
			if ( (!meteorRandomDecay) || (random(10)>5) ) {
				fadeToBlack(j, meteorTrailDecay );        
			}
		}
	 
		// draw meteor
		for (int j = 0; j < meteorSize; j++) {
			if ( ( i-j <NUM_LEDS) && (i-j>=0) ) {
				setPixel(i-j, red, green, blue);
			}
		}
	 
		FastLED.show();
		delay(SpeedDelay);
	}
	//fill the strip with the color starting from the end with the color 
	for (int i = NUM_LEDS; i >= 0; i-=3) {
		setPixel(i, red, green, blue);
		setPixel(i-1, red, green, blue);
		setPixel(i-2, red, green, blue);
		FastLED.show();
		delay(FillDelay);
	}
}


void meteorRainHalfFill(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay, int FillDelay) {  
	setAll(0, 0, 0);
 
	for (int i = 0; i < NUM_LEDS/2 + 30; i++) {
	 // start in center at num/2 spread out i from center each time 
	 
		// fade brightness all LEDs one step
		for (int j=0; j<NUM_LEDS; j++) {
			if ( (!meteorRandomDecay) || (random(10)>5) ) {
				fadeToBlack(j, meteorTrailDecay);        
			}
		}
	 
		// draw meteors
		for (int j = 0; j < meteorSize; j++) {
			if ( ( i-j <(NUM_LEDS/2)) && (i-j>=0) ) {
				setPixel((NUM_LEDS/2) - (i-j), red, green, blue);
			}
		}
		for (int j = 0; j < meteorSize; j++) {
			if ( ( i-j <(NUM_LEDS/2)) && (i-j>=0) ) {
				setPixel((NUM_LEDS/2) + (i-j), red, green, blue);
			}
		}
	 
		FastLED.show();
		delay(SpeedDelay);
	}
	//fill the strip with the color starting from the ends
	for (int i = 0; i <= NUM_LEDS/2; i+=2) {
		setPixel(i, red, green, blue);
		setPixel(NUM_LEDS - i, red, green, blue);
		setPixel(i+1, red, green, blue);
		setPixel(NUM_LEDS - i - 1, red, green, blue);
		FastLED.show();
		delay(FillDelay);
	}
}

void fadeToBlack(int ledNo, byte fadeValue) {
	leds[ledNo].fadeToBlackBy(fadeValue); 
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
	leds[Pixel].r = red;
	leds[Pixel].g = green;
	leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
	for (int i = 0; i < NUM_LEDS; i++ ) {
		setPixel(i, red, green, blue);
	}
	FastLED.show();
}