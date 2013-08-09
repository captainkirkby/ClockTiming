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
	uint16_t adcValue = analogRead(ANALOG_READ_PIN);		//THIS instruction takes a long time

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
