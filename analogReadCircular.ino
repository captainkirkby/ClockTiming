#include "wiring_private.h"
#include "pins_arduino.h"

//Set BaudRate
const uint16_t BAUD_RATE = 9600;

//Create buffer hardcoded with 800 elements
const uint16_t BUFFER_LENGTH = 800;
uint16_t circularbuffer[BUFFER_LENGTH];
uint16_t currentElementIndex;

//Set trigger
const uint16_t THRESHOLD = 500;

//Declare timer
uint16_t timer;
const uint16_t END_TIMER = 500;

//Set pins
const uint8_t DIGITAL_TRIGGER_PIN = 4;
const uint8_t DIGITAL_PULSE_PIN = 2;
const uint8_t ANALOG_READ_PIN = 0;

//virtual function declarations
//void dumpToSerialPort();
//void addToBuffer(uint16_t adcValue);

uint8_t analog_reference = DEFAULT;

void analogReference(uint8_t mode)
{
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
	analog_reference = mode;
}

void setup(){
	//Setup Digital output pins
	pinMode(DIGITAL_PULSE_PIN, OUTPUT);
	digitalWrite(DIGITAL_PULSE_PIN, LOW);

	//Setup Digital trigger pins
	pinMode(DIGITAL_TRIGGER_PIN, OUTPUT);
	digitalWrite(DIGITAL_TRIGGER_PIN, LOW);

	//Setup circular buffer
	currentElementIndex = 0;

	//Setup timer
	timer = 0;
}

void loop(){
	//__asm__("nop\n\t");	//No operation that takes 1 clock cycle.... Not so sure it works

	//Set pin to high
	digitalWrite(DIGITAL_PULSE_PIN, HIGH);

	//
	// uint16_t adcValue = analogRead(ANALOG_READ_PIN);		//THIS instruction takes a long time

	uint8_t low, high;

	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		if (pin >= 54) pin -= 54; // allow for channel or pin numbers
	#elif defined(__AVR_ATmega32U4__)
		if (pin >= 18) pin -= 18; // allow for channel or pin numbers
	#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || 
	defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
		if (pin >= 24) pin -= 24; // allow for channel or pin numbers
	#elif defined(analogPinToChannel) && (defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__))
		pin = analogPinToChannel(pin);
	#else
		if (pin >= 14) pin -= 14; // allow for channel or pin numbers
	#endif
		
	#if defined(__AVR_ATmega32U4__)
		pin = analogPinToChannel(pin);
		ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
	#elif defined(ADCSRB) && defined(MUX5)
		// the MUX5 bit of ADCSRB selects whether we're reading from channels
		// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
		ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
	#endif
	  
		// set the analog reference (high two bits of ADMUX) and select the
		// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
		// to 0 (the default).
	#if defined(ADMUX)
		ADMUX = (analog_reference << 6) | (ANALOG_READ_PIN & 0x07);
	#endif
	
		// without a delay, we seem to read from the wrong channel
		//delay(1);
	
	#if defined(ADCSRA) && defined(ADCL)
		// start the conversion
		sbi(ADCSRA, ADSC);
	
		// ADSC is cleared when the conversion finishes
		while (bit_is_set(ADCSRA, ADSC));
	
		// we have to read ADCL first; doing so locks both ADCL
		// and ADCH until ADCH is read.  reading ADCL second would
		// cause the results of each conversion to be discarded,
		// as ADCL and ADCH would be locked when it completed.
		low  = ADCL;
		high = ADCH;
	#else
	// 	// we dont have an ADC, return 0
		low  = 0;
		high = 0;
	#endif
	
		// combine the two bytes
	uint16_t adcValue = (high << 8) | low;

	if(timer > 0){
		//if we are still filling buffer before dump
		++timer;

		//Time padding
		digitalWrite(DIGITAL_TRIGGER_PIN, LOW);

		//check if we are done filling the buffer
		if(timer == END_TIMER){
			//uncomment this for mutiple buffers.
			timer = 0;

			//dump data gather session to serial port
			dumpToSerialPort();
		}
	} else {
		/*
		if above THRESHOLD, allow buffer to fill n number of times, dump on the serial port, then continue
		in either case, fill buffer once and increment pointer
		*/	
	
		if(adcValue >= THRESHOLD){
			//start timer
			timer = 1;
			digitalWrite(DIGITAL_TRIGGER_PIN, HIGH);


			//Waste a few clock cycles to note on the oscilloscope that we have started the timer
			//delayMicroseconds(1);
		} else {
			//make sure timer is stopped
			timer = 0;
			digitalWrite(DIGITAL_TRIGGER_PIN, LOW);
		}
	}

	//Add value to buffer
	currentElementIndex = (currentElementIndex + 1) % BUFFER_LENGTH;
	circularbuffer[currentElementIndex] = adcValue;

	//Set pin to low
	digitalWrite(DIGITAL_PULSE_PIN, LOW);
}

//Dumps the circular buffer from current element, wrapping around, to itself.
void dumpToSerialPort(){
	//Initialize serial communication maybe do this only when needed
	Serial.begin(BAUD_RATE);
	
	for(int i=currentElementIndex;i<BUFFER_LENGTH;++i){
		//Serial.print(i);
		//Serial.print(": ");
		Serial.println(circularbuffer[i]);
	}
	for(int i=0;i<currentElementIndex;++i){
		//Serial.print(i);
		//Serial.print(": ");
		Serial.println(circularbuffer[i]);
	}
}

// //Adds to the next part of the circular buffer
// void addToBuffer(uint16_t adcValue){
// 	currentElementIndex = (currentElementIndex + 1) % BUFFER_LENGTH;
// 	circularbuffer[currentElementIndex] = adcValue;
// }
