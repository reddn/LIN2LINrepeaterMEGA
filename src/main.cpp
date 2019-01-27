#include <Arduino.h>
#include <CustomSoftwareSerial.h>

#define Serialwrite Serial1.write
// #define Serialwrite customSerial->write  //on change fix setup
#define DEBUG 1

CustomSoftwareSerial* customSerial;

uint8_t lkas_off_array[][4] =  {{0x20,0x80,0xc0,0xa0},{0x00,0x80,0xc0,0xc0}};
uint8_t counterbit = 0;
uint8_t buff[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t buffi = 0;
bool lkas_active = false;
uint8_t errorcount = 0;
uint8_t sendlast = 0;
uint8_t buffilastsent = 0xff;
uint8_t createdMsg[4][5]; //4 buffers of 5 bytes, the 4 bytes is the frame, the 5th byte is the index index off buff where the frame was made from (note the buff is 2 bytes)
uint8_t lastCreatedMsg = 0xff;
uint8_t lastCreatedMsgSent = 0xff;
volatile uint8_t  sendArrayFlag = 0;
int16_t rotaryCounter = 0;  // min is 0 max is 906 -- default sent to center
uint16_t mainCounter = 0;
uint8_t forceRotaryAsInput = 0;
uint8_t forceLkasActive = 0;
uint8_t forceLkasActive_prev = 0;
uint16_t serialDebugSendCounter = 0;
int16_t centerPoint = 450; //default for center is 450 (max on this pot is 906)
bool sendDebug = false;
unsigned long pin13LedLastChange =0;
uint8_t pin13LedStatus = 0;
uint8_t useOP2byteSerialRX = 1;
uint8_t MCUSerialData[4];
uint8_t MCUSerialDataPos = 1;
unsigned long lastTimeSerialSent = 0;

void readSettingsPins();
void sendLKASOffArray();
void sendLKASOnArray();
void sendSerialDebugDataOverSerial(uint8_t*);
void sendArray(uint8_t*);
void createSerialMsg(uint8_t*, uint8_t*);
void putByteInNextBuff(uint8_t *msgnum, uint8_t *temp);
void handleRotary();
void checkAndRunSendArrayFlag();
void checkForRightCounterInPreCreatedMsg();
void checkOPSerialRxInput();
void createAndSendSerialMsgUsingRotary();
void changeCenterPoint();
void pin13LedFunc();
void printuint_t(uint8_t);
void checkMCUSerialRxInput();
uint8_t chksm(uint8_t*);
uint8_t chksm(uint16_t*);
void setupTimersOld();
void sendMCUSerial();
void setupTimer1();

// all input pins are active low (except rotary pot input)
//7: Force LKAS ON  - 12: set Rotary POT current pos as center  -
// 13: used for LED when LKAS is on  - A0: Enable Rotary as INPUT  -
// A4: send sendDebug on serial  - A5: rotary pot input
void setup() {
	Serial.begin(115200);
	// customSerial = new CustomSoftwareSerial(9, 10, true); // rx, tx, inverse logic (default false)
	// customSerial->begin(9600, CSERIAL_8E1);         // Baud rate: 9600, configuration: CSERIAL_8N1
  Serial1.begin(9600,SERIAL_8E1);
  Serial2.begin(9600,SERIAL_8E1);
  Serial3.begin(9600,SERIAL_8E1);
	cli();  // no interrupts
	setupTimer1();
	// setupTimersOld();
	sei();//allow interrupts
	pinMode(12, INPUT_PULLUP);
	pinMode(7, INPUT_PULLUP); //Force LKAS Active
	pinMode(13,OUTPUT);
	pinMode(A0, INPUT_PULLUP);  //use Rotary encoder as input -- active low
	pinMode(A4, INPUT_PULLUP);  //enable sending debuginfo on hardware Serial TX
	pinMode(A3, INPUT_PULLUP);  //enable full ADAS MCU LIN frame buffering/repeating
	digitalWrite(13,LOW);

	readSettingsPins();

}

/*** OP to LIN2LIN data structure (it still sends 4 bytes, but the last 2 are 0x00).
//i will prolly change that later to only 2 bytes, but this code will not need to be changed
// b01A0####    ####is big_steer   A = Active   first 2 bits is the byte counter
// b10A#####    ##### is little steer  ***/
void loop() {
	if(useOP2byteSerialRX){
		checkForRightCounterInPreCreatedMsg();
		// checkAndRunSendArrayFlag(); //read seteting pin
	}
	handleRotary();
  readSettingsPins();
	if(useOP2byteSerialRX) {
		// checkAndRunSendArrayFlag();
		checkOPSerialRxInput(); //modify this... use to be useSerialRxAsInput
	}
	else checkMCUSerialRxInput();
	readSettingsPins();
  pin13LedFunc();
}  // end of loop

void checkAndRunSendArrayFlag(){

	if(errorcount > 1) {
		lkas_active = false;
		errorcount = 0;
		// sendArrayFlag = 0;
		if(sendDebug) Serial.println("error greater than1..");
	}
	// if(sendArrayFlag){
		if(lkas_active){ //if lkas_active need to send the live data, if not, send
			sendLKASOnArray();
		}else {
			sendLKASOffArray();
		}

		// sendArrayFlag = 0;
	// }
}
 					/***  LOOP directly called functions ***/

void checkOPSerialRxInput(){
	while(Serial.available()){
		uint8_t temp = Serial.read();
		if((temp >> 6) <3)
		// Serial.print(temp);
		if(serialDebugSendCounter % 151 || serialDebugSendCounter % 150){
			printuint_t(temp);
		}
		if(temp == 0x00) break;
		uint8_t active =  (temp >> 5) & 0x01; //if bb1bbbbb ... 1 is active bit
		if(!active) {
			lkas_active = false;
			break;
		}
		uint8_t msgnum = (temp >> 6); //its 1 or 2
		putByteInNextBuff(&msgnum,&temp);
	} // end of while
}



void sendLKASOffArray(){
	Serialwrite(lkas_off_array[counterbit][0]);
	Serialwrite(lkas_off_array[counterbit][1]);
	Serialwrite(lkas_off_array[counterbit][2]);
	Serialwrite(lkas_off_array[counterbit][3]);
	if(sendDebug)	sendSerialDebugDataOverSerial((uint8_t*)&lkas_off_array[counterbit][0]);
	counterbit = counterbit > 0 ? 0x00 : 0x01;
	// if(counterbit) counterbit = 0x00; else counterbit = 0x01;
}

void sendLKASOnArray(){
	if(forceRotaryAsInput){
		createAndSendSerialMsgUsingRotary();
		counterbit = counterbit > 0 ? 0x00 : 0x01;
		return;
	}
	if(lastCreatedMsg == 0xff) {
		errorcount = 0x10;
		return;
	}
	if(lastCreatedMsg == lastCreatedMsgSent){  //TODO: allow 1 resend of last data, but needs to be recreated w new counter /checksum
		errorcount++;
		createSerialMsg(&createdMsg[lastCreatedMsgSent][4], &lastCreatedMsg);
	} else errorcount = 0;
	Serialwrite(createdMsg[lastCreatedMsg][0]);
	Serialwrite(createdMsg[lastCreatedMsg][1]);
	Serialwrite(createdMsg[lastCreatedMsg][2]);
	Serialwrite(createdMsg[lastCreatedMsg][3]);
	lastCreatedMsgSent = lastCreatedMsg;
	if(sendDebug)	sendSerialDebugDataOverSerial((uint8_t*)&createdMsg[lastCreatedMsg][0]);
	counterbit = counterbit > 0 ? 0x00 : 0x01;
}

void checkForRightCounterInPreCreatedMsg(){
	if(lastCreatedMsg != lastCreatedMsgSent){
		if((createdMsg[lastCreatedMsg][0] >> 5) != counterbit){
			createSerialMsg(&createdMsg[lastCreatedMsg][4], &lastCreatedMsg);
		}
	}
}

void handleRotary(){  // min is 0 max is 906 of A5 using the rotary
	rotaryCounter = (analogRead(A5) - centerPoint) / 3;  //centerPoint is 450   //new center is 0. neg is left, pos is right
	if(centerPoint != 450){
		if(rotaryCounter > 255) rotaryCounter = 255;
		if(rotaryCounter < -255) rotaryCounter = -255;
	}
}  //Need to scale the center so its not soo touchy.

void checkMCUSerialRxInput(){
	while(Serial.available()){
		uint8_t tempdata;
		tempdata = Serial.read();
		if(MCUSerialDataPos == 0){  //start at 1 so it equals the last 2 bi
			if((tempdata >> 5) < 2) {
				counterbit = tempdata >> 5; //keeps counter bits aligned
				MCUSerialData[0] = tempdata;
				MCUSerialDataPos++;
			}
			else return;  //if on dataPosition 0, and data is not < 2.. return, somethings fuckedup
		} else{
			MCUSerialData[MCUSerialDataPos] = tempdata;
			MCUSerialDataPos++;
			if(MCUSerialDataPos == 2) {
				MCUSerialDataPos = 0;
				if(serialDebugSendCounter %151){
					Serial.print("R ");
					printuint_t(MCUSerialData[0]);
					Serial.print(" ");
					printuint_t(MCUSerialData[1]);
					Serial.println("");
				}
				sendMCUSerial();
			}
		}
	}//end While
} // end function

void putByteInNextBuff(uint8_t *msgnum, uint8_t *temp){
	uint8_t buffievenodd = buffi % 2; //buffi is the next byte we are expecting, count starts at 0 .  ie  byte 1 counter is 1, goes in an even buffi (starting at 0)
	switch(*msgnum){
		case 1:  //byte counter from serial message starts at 1
		if(buffievenodd == 1){  //something went wrong with last send, just reset everthing and increase error count
			errorcount++;
			buff[buffi-1] = *temp;
		} else {
			if(buffi > 4) buffi = 0; //this should nto happen, just in case tho
			buff[buffi] = *temp;
			buffi++;
		}
		break;
		case 2: //byte counter starts at 1. so counter == 2 is the 2nd byte
		if(buffievenodd == 0){  //something went wrong with last send, just reset everthing and increase error count.
			errorcount++; //expecting the 1st byte not 2nd, add error and return
		} else {
			buff[buffi] = *temp;
			uint8_t buffisub = buffi - 1;
			lastCreatedMsg = lastCreatedMsg < 3 ? lastCreatedMsg + 1 : 0;
			createSerialMsg(&buffisub, &lastCreatedMsg);
			if(buffi <5) buffi++; else buffi = 0;//assign buffi the next even, unless its 5, then go to 0
		}
		break;
	}
}

void readSettingsPins(){
	forceRotaryAsInput = !digitalRead(A0);
	forceLkasActive = !digitalRead(7);
	sendDebug = !digitalRead(A4);
	useOP2byteSerialRX = digitalRead(A3);
	if(forceLkasActive_prev != forceLkasActive) {
		lkas_active = forceLkasActive;
		forceLkasActive_prev = forceLkasActive;
	}
	if(!digitalRead(12)) 	centerPoint = analogRead(A5);
}


void createAndSendSerialMsgUsingRotary(){
	uint8_t data[4] = {0x00,0x00,0x00,0x00};
	data[0] = (counterbit << 5) | ((rotaryCounter >> 12) & 0x8) | ((rotaryCounter >> 5) & 0xF);
	data[1] = 0xA0 | (rotaryCounter & 0x1F);
	data[2] =  0x80;
	Serialwrite(data[0]);
	Serialwrite(data[1]);
	uint16_t total = data[0] + data[1] +   data[2];
	Serialwrite(data[2]);
	data[3] = chksm(&total);
	Serialwrite(data[3]);
	if(sendDebug)	sendSerialDebugDataOverSerial((uint8_t*)&data[0]);
}

void createSerialMsg(uint8_t *localbuffi, uint8_t *msgi){ //array index of buff (even)
	createdMsg[*msgi][0] = (buff[*localbuffi] & 0xF) | (counterbit << 0x5);
	createdMsg[*msgi][1] = (buff[*localbuffi+1] & 0x1F) | 0xA0; //cratedMsgmsg [2] is hard set to 0x80
	createdMsg[*msgi][2] = 0x80;
	createdMsg[*msgi][3] = chksm((uint8_t*)&msgi);
	createdMsg[*msgi][4] = *localbuffi;
	lastCreatedMsg = *msgi;
}

void sendSerialDebugDataOverSerial(uint8_t* thisdata){
	if((serialDebugSendCounter % 15)== 0){
		printuint_t(thisdata[0]);
		Serial.print(" ");
		printuint_t(thisdata[1]);
		Serial.print(" ");
		printuint_t(thisdata[2]);
		Serial.print(" ");
		printuint_t(thisdata[3]);
		Serial.print(" ");
		int16_t apply_steer = ((thisdata[0] & 0x7)<<5) | ((thisdata[0] & 0x8) << 12) |
				(thisdata[1] & 0xF);
		if((thisdata[0] & 8) > 0) apply_steer = apply_steer | 0x7F00;
		Serial.print(apply_steer, DEC);
		Serial.print(" --a ");
		Serial.print(rotaryCounter, DEC);
		Serial.print(" t ");
		Serial.println(micros() - lastTimeSerialSent, DEC);
	}
	serialDebugSendCounter++;
	lastTimeSerialSent = micros();
}

void printuint_t(uint8_t var) {
  for (uint8_t test = 0x80; test; test >>= 1) {
    Serial.write(var  & test ? '1' : '0');
  }
  // Serial.println();
}

void pin13LedFunc(){
	if((millis() - pin13LedLastChange) > 299){
		if(lkas_active){
			pin13LedStatus = !pin13LedStatus;
			digitalWrite(13, pin13LedStatus);
		}
		else digitalWrite(13,LOW);
		pin13LedLastChange = millis();
	}
}

// make unsigned long lastTimeMCUSerialSent
// uint8_t MCUSerialData[4]
// uint8_t MCUSerialDataPos = 0

void sendMCUSerial(){
	pin13LedFunc();
	if(forceRotaryAsInput){
		createAndSendSerialMsgUsingRotary();
	}else {
		uint8_t _msgi = 0;
		uint8_t _active =  (MCUSerialData[1] >> 5) & 0x1;
		createdMsg[0][0] = MCUSerialData[0];
		createdMsg[0][1] = MCUSerialData[1];
		Serialwrite(createdMsg[0][0]);
		Serialwrite(createdMsg[0][1]);
		createdMsg[0][2] = _active == 1 ? 0x80 : 0xc0;
		createdMsg[0][3] = chksm((uint8_t*)&_msgi);
		Serialwrite(createdMsg[0][2]);
		Serialwrite(createdMsg[0][3]);
		if(sendDebug)	sendSerialDebugDataOverSerial((uint8_t*)&createdMsg[0][0]);

	}

}





					/*** CHECKSUMS ***/
uint8_t chksm(uint8_t *msgi){
	uint16_t local = createdMsg[*msgi][0] + createdMsg[*msgi][1] + createdMsg[*msgi][2] ;
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

uint8_t chksm(uint16_t *input){
	uint16_t local = *input % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

					/*** TIMERS AND INTERRUPT FUNCTS ***/

ISR(TIMER2_COMPA_vect) {
  // sendArrayFlag = 1;
  checkAndRunSendArrayFlag();
}//
ISR(TIMER1_COMPA_vect){
	// sendArrayFlag = 1;
	checkAndRunSendArrayFlag();
}//
ISR(TIMER0_COMPA_vect){sendArrayFlag = 1;}//

void setupTimersOld(){
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 155 ;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256)  177== 11.4ms apart 87.2 hz
  // turn on CTC mode
  TCCR2A |= 0b10; //(1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= 0b111; //(1 << CS12) | (1 << CS10;
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void setupTimer1WebSite(){
	TCCR2A = 0; // set entire TCCR2A register to 0
	TCCR2B = 0; // same for TCCR2B
	TCNT2  = 0; // initialize counter value to 0
	// set compare match register for undefined Hz increments
	OCR2A = 0x80 ;
	// turn on CTC mode
	TCCR2B |= (1 << WGM21);
	// Set CS22, CS21 and CS20 bits for 1 prescaler
	TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

}

void setupTimer1(){
	TIMSK1 |= (1 << OCIE1A);
  TCCR1A = 0;// set entire TCCR2A register to 0
  TCCR1B = 0;// same for TCCR2B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR1A = 20000 ;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256) // mod from 3690 to half, then 66%
  // turn on CTC mode
  TCCR1A |= 0b00; //(1 << WGM21);    wgm 2 = 1 , 1= 0, 0=0
  // Set CS21 bit for 1024 prescaler
  TCCR1B |= 0b1010; //8 prescale (1 << CS11) | (1 << CS10);

  // enable timer compare interrupt
}

void setupTimer2(){
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 236;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256)
	OCR2B = 236;
	// turn on CTC mode
  TCCR2A |= 0x2;
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= 0b111; //(1 << CS11) | (1 << CS10);

  // enable timer compare interrupt
  TIMSK2 |= 0x80;
}
