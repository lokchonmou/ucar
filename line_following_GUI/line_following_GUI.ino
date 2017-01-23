//--------define the pins of robot, change it if you need-------
byte E1 = 5;  					//LEFT MOTOR
byte M1 = 4; 
byte E2 = 6;                    //RIGHT MOTOR
byte M2 = 7; 
byte button_array_pin = A3;
byte MaxSpeed = 127;
byte SetMaxSpeed = 127;
byte sensor_pin[] = {10,11,12,13,A0, A1, A2};
byte IR_pin = 9;
byte Motor_wiring_count;
unsigned long long_press_counter;
boolean long_press_enable;
// you alse need a LiquidCrystal_I2C library for the I2C LCD
//---------------------------------------------


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

boolean sensor_state[] = {0, 0, 0, 0, 0, 0, 0}; //save the boolean value of edge sensor
byte Map = 0;			//convert the above boolean to a byte, easy to compare

unsigned long timer[] = {0,0};
byte page = 0;		//page number of GUI

float input, output, Kp, Kd, lastinput, SampleTime;
unsigned long lastTime;		//use to control the car
byte IR_brightness;			//Brightness of IR LED
byte B_W;						//0: detect white line, 1: detect black line

byte ENTER[8] = {
	0b00000,
	0b00001,
	0b00001,
	0b00101,
	0b01001,
	0b11111,
	0b01000,
	0b00100
};
byte BACK[8] = {
	0b00000,
	0b00100,
	0b01000,
	0b11111,
	0b10000,
	0b01000,
	0b00100,
	0b00000
};
byte GO[8] = {
	0b00000,
	0b00100,
	0b00010,
	0b11111,
	0b00010,
	0b00100,
	0b00000,
	0b00000
};

void setup()
{
	lcd.init();                       
	lcd.backlight();
	lcd.createChar(0, ENTER);
	lcd.createChar(1, BACK);
	lcd.createChar(2, GO);
	pinMode(M1, OUTPUT);   
	pinMode(M2, OUTPUT); 
	pinMode(E1, OUTPUT);   
	pinMode(E2, OUTPUT); 
	pinMode(button_array_pin, INPUT);
	pinMode(IR_pin, OUTPUT);
	for(int i=0; i<=6; i++) pinMode(sensor_pin[i], INPUT_PULLUP);
	lcd.setCursor(0,0);
	lcd.print("   PD control   ");
	lcd.setCursor(0,1);
	lcd.print(" line following ");
	delay(2000);
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Initializing...");
	EEPROM.get(0, Kp);
	EEPROM.get(10, Kd);
	EEPROM.get(20, IR_brightness);
	EEPROM.get(30, B_W);
	EEPROM.get(40, SampleTime);
	EEPROM.get(50, SetMaxSpeed);
	EEPROM.get(60, Motor_wiring_count);

	delay(1000);

	if(isnan(Kp))	Kp = 40.0;
	if(isnan(Kd))	Kd = 0.0;
	if(isnan(SampleTime))	SampleTime = 30;
  if(isnan(IR_brightness)) IR_brightness = 255;
	if(B_W == 255)	B_W = 0;
}

void loop()
{
	Kp = constrain(Kp, 0.0, 255.0);
	Kd = constrain(Kd, 0.0, 10.0);
	analogWrite(IR_pin, IR_brightness);
	GUI();
	//Read the edge sensor and put it in a byte
	{
		Map = 0;
		for (int i = 0; i <= 6; i++)
		{
			if (B_W == LOW)	sensor_state[i] = !digitalRead(sensor_pin[i]);
			else	sensor_state[i] = digitalRead(sensor_pin[i]);
			Map = Map | sensor_state[i];
			Map = Map << 1; 
		}
	}
	//Convert the sensor to a scale value
	//change the sensorMap to input
	{
		switch (Map)
		{
			case B10000000:
			input = -6;
			break;
			case B11000000:
			input = -5;
			break;
			case B01000000:
			input = -4;
			break;
			case B01100000:
			input  = -3;
			break;
			case B00100000:
			input  = -2;
			break;
			case B00110000:
			input  = -1;
			break;
			case B00010000:
			input  = 0;
			break;
			case B00011000:
			input  = 1;
			break;
			case B00001000:
			input  = 2;
			break;
			case B00001100:
			input  = 3;
			break;
			case B00000100:
			input  = 4;
			break;
			case B00000110:
			input  = 5;
			break;
			case B00000010:
			input  = 6;
			break;
		}
	}
	// PD control part
	{
		unsigned long now = millis();
		unsigned long timeChange = (now - lastTime);
		if (timeChange >= SampleTime)
		{
			output = -Kp*input- Kd*(input - lastinput)/(SampleTime*1e-3);
			output = constrain(output, -2*MaxSpeed, 2*MaxSpeed);
			drive();

			lastTime = now;
			lastinput = input;
		}
	}
}

void GUI(){
	int button_reading = analogRead(button_array_pin);
	if(page != 0){
		MaxSpeed = 0;
	}
	else {
		MaxSpeed = SetMaxSpeed;
	}

	if(millis() - timer[0] >= 200){
		if(button_reading <= 150){	
			long_press_counter = millis();
			long_press_enable = HIGH;

			if(page < 9 && page >= 1)	page++;
			else if(page == 9)	page = 1;

			else if(page == 21)	Kp++;
			else if(page == 22)	Kd+=0.1;
			else if(page == 23)	IR_brightness+=1;
			else if(page == 24)	SampleTime+=10;
			else if(page == 25) {
				Motor_wiring_count++; 
				if(Motor_wiring_count>8) Motor_wiring_count=0;
			}
			else if(page == 26)	SetMaxSpeed+=1;
			timer[0] = millis();
		}
		else if(button_reading >=150 && button_reading <= 600){		
			long_press_counter = millis();
			long_press_enable = HIGH;

			if(page == 1)	page = 9;
			else if(page <= 9 && page > 1)	page--;

			else if(page == 21)	Kp--;
			else if(page == 22)	Kd-=0.1;
			else if(page == 23)	IR_brightness-=1;
			else if(page == 24)	SampleTime-=10;
			else if(page == 25) {
				Motor_wiring_count--;
				if(Motor_wiring_count<0) Motor_wiring_count=7;
			}
			else if(page == 26)	SetMaxSpeed-=1;
			timer[0] = millis();
		}
		if(long_press_enable == HIGH && millis()-long_press_counter >= 3000){		//joystick pressed
			long_press_enable = LOW;
			if(page == 0)		page = 1;
			else if(page == 1)	page = 0;
			else if(page == 2)	page = 21;
			else if(page == 3)	page = 22;
			else if(page == 4)	page = 23;
			else if(page == 5)	{
				EEPROM.put(0, Kp);
				EEPROM.put(10, Kd);
				EEPROM.put(20, IR_brightness);
				EEPROM.put(30, B_W);
				EEPROM.put(40, SampleTime);
				EEPROM.put(50, SetMaxSpeed);
				EEPROM.put(60, Motor_wiring_count);
			}
			else if(page == 6)	{if(B_W == 0) B_W=1; else if(B_W == 1) B_W=0; else B_W = 0;}
			else if(page == 7)	page = 24;
			else if(page == 8)	page = 25;
			else if(page == 9)	page = 26;
			else if(page == 21)	page = 2;
			else if(page == 22)	page = 3;
			else if(page == 23)	page = 4;
			else if(page == 24) page = 7;
			else if(page == 25) page = 8;
			else if(page == 26) page = 9;
			timer[0] = millis();
		}
	}

	switch (page) {
		case 0:							// info page
		{
			lcd.setCursor(0,0);
			lcd.print("Kp=" + String(Kp,1) + "  Kd=" + String(Kd,3));
			lcd.setCursor(0,1);
			lcd.print("sen=");
			for(int i=0; i<=6; i++)	lcd.print(sensor_state[i]);
			lcd.print(" ");
			lcd.print(String(input,1));
		}
		break;
		case 1:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (1/9)");
			lcd.setCursor(0,1);
			lcd.print(" INFO SCREEN");
			lcd.write((uint8_t)0);
			lcd.print("  ");
			lcd.write((uint8_t)2);

		}
		break;
		case 2:{							
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (2/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" Tune Kp");
			lcd.write((uint8_t)0);
			lcd.print("     ");
			lcd.write((uint8_t)2);
		}
		break;
		case 3:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (3/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" Tune Kd");
			lcd.write((uint8_t)0);
			lcd.print("     ");
			lcd.write((uint8_t)2);
		}
		break;
		case 4:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (4/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" Tune IR LED");
			lcd.write((uint8_t)0);
			lcd.print(" ");
			lcd.write((uint8_t)2);
		}
		break;
		case 5:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (5/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" SAVE DATA");
			lcd.write((uint8_t)0);
			lcd.print("   ");
			lcd.write((uint8_t)2);
		}
		break;
		case 6:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (6/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" B/W LINE");
			lcd.write((uint8_t)0);
			lcd.print("  ");
			if(B_W == 1)	lcd.print("B ");
			else if(B_W == 0) lcd.print("W ");
			else lcd.print("ERROR");
			lcd.write((uint8_t)2);
		}
		break;
		case 7:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (7/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" SIMPLE TIME");
			lcd.write((uint8_t)0);
			lcd.print(" ");
			lcd.write((uint8_t)2);
		}
		break;
		case 8:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (8/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" MOTOR DIR  ");
			lcd.write((uint8_t)0);
			lcd.print(" ");
			lcd.write((uint8_t)2);
		}
		break;
		case 9:{
			lcd.setCursor(0,0);
			lcd.print("  MENU:    (9/9)");
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print("MOTOR SPEED ");
			lcd.write((uint8_t)0);
			lcd.print(" ");
			lcd.write((uint8_t)2);
		}
		break;
		case 21:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print("  Kp = " + String(Kp,3)+"     ");
			lcd.setCursor(15,0);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" BACK");
			lcd.write((uint8_t)0);
			lcd.print("         ");
		}
		break;
		case 22:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print("  Kd = " + String(Kd,3)+"     ");
			lcd.setCursor(15,0);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" BACK");
			lcd.write((uint8_t)0);
			lcd.print("         ");
		}
		break;
		case 23:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print(" IR =   " + String(IR_brightness) + "        ");
			lcd.setCursor(0,15);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.print("sensor:");
			for(int i=0; i<=4; i++)	lcd.print(sensor_state[i]);
			lcd.print(" ");
			lcd.print(String(input,1));
		}
		break;
		case 24:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print(" SIMPLE T=" + String(SampleTime) + "  ");
			lcd.setCursor(0,15);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" BACK");
			lcd.write((uint8_t)0);
			lcd.print("         ");
		}
		break;
		case 25:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print("  MOTOR DIR   ");
			lcd.setCursor(0,15);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			if(Motor_wiring_count == 0)	lcd.print(" 1.L+,L-,R+,R-");
			if(Motor_wiring_count == 1)	lcd.print(" 2.L-,L+,R+,R-");
			if(Motor_wiring_count == 2)	lcd.print(" 3.L+,L-,R-,R+");
			if(Motor_wiring_count == 3)	lcd.print(" 4.L-,L+,R-,R+");
			if(Motor_wiring_count == 4)	lcd.print(" 5.R+,R-,L+,L-");
			if(Motor_wiring_count == 5)	lcd.print(" 6.R-,R+,L+,L-");
			if(Motor_wiring_count == 6)	lcd.print(" 7.R+,R-,L-,L+");
			if(Motor_wiring_count == 7)	lcd.print(" 8.R-,R+,L-,L+");
			lcd.write((uint8_t)0);
		}
		break;
		case 26:{
			lcd.setCursor(0,0);
			lcd.write((uint8_t)1);
			lcd.print("Max Speed =" + String(SetMaxSpeed)+"  ");
			lcd.setCursor(15,0);
			lcd.write((uint8_t)2);
			lcd.setCursor(0,1);
			lcd.write((uint8_t)1);
			lcd.print(" BACK");
			lcd.write((uint8_t)0);
			lcd.print("         ");
		}
		break;
	}
}

void drive(){
	if(Motor_wiring_count == 0)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, MaxSpeed - abs(output));   
			analogWrite(E2, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, abs(output) - MaxSpeed);   
			analogWrite(E2, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, HIGH);   
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 1)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, MaxSpeed - abs(output));   
			analogWrite(E2, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, LOW);       
			analogWrite(E1, abs(output) - MaxSpeed);   
			analogWrite(E2, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, HIGH);   
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 2)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, MaxSpeed - abs(output));   
			analogWrite(E2, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, abs(output) - MaxSpeed);   
			analogWrite(E2, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, LOW);   
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 3)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, MaxSpeed - abs(output));   
			analogWrite(E2, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M1,LOW);   
			digitalWrite(M2, HIGH);       
			analogWrite(E1, abs(output) - MaxSpeed);   
			analogWrite(E2, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M1,HIGH);   
			digitalWrite(M2, LOW);   
			analogWrite(E1, MaxSpeed);   
			analogWrite(E2, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 4)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, MaxSpeed - abs(output));   
			analogWrite(E1, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, abs(output) - MaxSpeed);   
			analogWrite(E1, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, HIGH);   
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 5)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, MaxSpeed - abs(output));   
			analogWrite(E1, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, LOW);       
			analogWrite(E2, abs(output) - MaxSpeed);   
			analogWrite(E1, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, HIGH);   
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 6)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, MaxSpeed - abs(output));   
			analogWrite(E1, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, abs(output) - MaxSpeed);   
			analogWrite(E1, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, HIGH);   
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, abs(output) - MaxSpeed);   
		}
	}
	if(Motor_wiring_count == 7)
	{
		if (output > -MaxSpeed && output <0)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, MaxSpeed - abs(output));   
		}
		else if(output >= 0 && output < MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, MaxSpeed - abs(output));   
			analogWrite(E1, MaxSpeed);   
		}
		else if(output >= MaxSpeed && output <= 2*MaxSpeed)
		{
			digitalWrite(M2,LOW);   
			digitalWrite(M1, HIGH);       
			analogWrite(E2, abs(output) - MaxSpeed);   
			analogWrite(E1, MaxSpeed); 
		} 
		else if(output >= -2*MaxSpeed && output <= -MaxSpeed)
		{
			digitalWrite(M2,HIGH);   
			digitalWrite(M1, LOW);   
			analogWrite(E2, MaxSpeed);   
			analogWrite(E1, abs(output) - MaxSpeed);   
		}
	}
}
