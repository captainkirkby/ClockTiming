//Set BaudRate
uint16_t baudRate = 9600;

//Trigger state
bool triggered = false;

//Create buffer hardcoded with 800 elements
const uint16_t bufferLength = 800;
uint16_t circularbuffer[bufferLength];
uint16_t currentElementIndex;

//Set trigger
const uint16_t THRESHOLD = 500;

//Declare timer
uint16_t timer;

//Set pins
const uint8_t digitalPulsePin = 2;
const uint8_t analogReadPin = 0;

//virtual function declarations
//void dumpToSerialPort();
//void addToBuffer(uint16_t adcValue);

void setup(){
	//Setup Digital output pins
	pinMode(digitalPulsePin, OUTPUT);
	digitalWrite(digitalPulsePin, LOW);

	//Setup circular buffer
	currentElementIndex = 0;
}

void loop(){
	//Set pin to high
	digitalWrite(digitalPulsePin, HIGH);

	uint16_t adcValue = analogRead(analogReadPin);		//THIS instruction takes a long time (~10ms!)

	if(triggered && timer > 0){
		//if we are still filling buffer before dump
		--timer;
		addToBuffer(adcValue);
	} else if(triggered && timer <= 0){
		//time to dump data and reset to original state
		triggered = false;
		dumpToSerialPort();
	} else {
		/*
		if above THRESHOLD, allow buffer to fill n number of times, dump on the serial port, then continue
		in either case, fill buffer once and increment pointer
		*/	
	
		if(adcValue >= THRESHOLD){
			//set "timer" to fill buffer n times
			triggered = true;
			timer = 500;		//n=500
		}

		addToBuffer(adcValue);

		//Set pint to low
		digitalWrite(digitalPulsePin, LOW);
	}
}

//Dumps the circular buffer from current element, wrapping around, to itself.
void dumpToSerialPort(){
	//Initialize serial communication maybe do this only when needed
	Serial.begin(baudRate);
	
	for(int i=currentElementIndex;i<bufferLength;++i){
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

//Adds to the next part of the circular buffer
void addToBuffer(uint16_t adcValue){
	if(currentElementIndex + 1 >= bufferLength){
		currentElementIndex = 0;
	} else {
		currentElementIndex = currentElementIndex;
	}

	circularbuffer[currentElementIndex] = adcValue;
	++currentElementIndex;
}
