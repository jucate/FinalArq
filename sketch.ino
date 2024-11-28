#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include "DHT.h"
#include <LiquidCrystal.h>
#include <Keypad.h>

#define LED_GREEN 42
#define LED_BLUE 44
#define LED_RED 47
#define PIR_SENSOR 10


uint64_t value = 0;


char password[5];
unsigned char idx = 0;

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {24, 26, 28, 30}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {32, 34, 36}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);





#define DHTPIN 46     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);


#define DEBUG(a) Serial.print(millis()); Serial.print(": "); Serial.println(a);

#define analogPin A0 //the thermistor attach to
#define beta 3950 //the beta of the thermistor

float tempC = 0.0;
// State Alias
enum State
{
	INICIO = 0,
	BLOQUEADO = 1,
	MONITOREO = 2,
	EVENTOS = 3,
  ALARMA = 4
};

// Input Alias
enum Input
{
	Sign_T = 0,
	Sign_P = 1,
	Sign_S = 2,
	Unknown = 3,
};

// Create new StateMachine
StateMachine stateMachine(5, 8);

// Stores last user input
Input input;


void read_Temperatura(void){
	Serial.println("TEMP_DIGITAL");
	// Read temperature as Celsius (the default)
  tempC = dht.readTemperature();
	Serial.print("TEMP: ");
	Serial.println(tempC);
	Serial.println("TEMP_ANALOG");
	//read thermistor value
  long a = analogRead(analogPin);
  //the calculating formula of temperature
  tempC = 1 / (log(1 / (1023.0 / a - 1)) / beta + 1.0 / 298.15) - 273.15;
	Serial.print("TEMP: ");
	Serial.println(tempC);
	if(tempC > 28.0){
		input = Input::Sign_P;
	}
}

float Hum;
void read_Humedad(void){
  Hum = dht.readHumidity(); 
}

void funct_Timeout(void){
  delay(7000);
  input = Input::Sign_T; 
}
void funct_Timeout2(void){
  delay(5000);
  input = Input::Sign_T; 
}
void funct_Timeout3(void){
  delay(3000);
  input = Input::Sign_T; 
}

#define PIN_IR 48
#define PIN_HALL 50

unsigned char valIR = LOW;
unsigned char valHall = LOW;

void read_Sensores(void){
  valIR = digitalRead(PIN_IR);
  valHall = digitalRead(PIN_HALL);

	if((valIR == HIGH) || (valHall == HIGH)){
		input = Input::Sign_S;
	}
	if(valIR == LOW){
		Serial.print("IR: ");
		Serial.println("No motion");
	}else{
		Serial.print("IR: ");
		Serial.println("Motion");
	}
	if(valHall == LOW){
		Serial.print("HALL: ");
		Serial.println("Light");
	}else{
		Serial.print("HALL: ");
		Serial.println("Dark");
	}
	
}

char key = 0;
void funct_keypad(void){
	key = keypad.getKey();
  
  if (key){
    Serial.println(key);
  }
}


// Variables para la contraseña
  char correctPassword[] = "1234"; // Define tu contraseña aquí
  char enteredPassword[5]; // Array para almacenar la contraseña ingresada (4 dígitos + terminador)
  int attempts = 0; // Contador de intentos
bool checkPassword(char *password) {
  return strcmp(password, correctPassword) == 0; // Compara la contraseña ingresada con la correcta
}
void reset() {
    idx = 0; // Reiniciar el índice
    lcd.clear();
    lcd.print("Ingresa clave:");
    lcd.setCursor(0, 1);
} 

void read_Password(void){
  char key = keypad.getKey();
 
  if (key) {
    if (key == '#') { // Al presionar '#', verifica la contraseña
      enteredPassword[idx] = '\0'; // Termina la cadena
      if (checkPassword(enteredPassword)) {
        lcd.clear();
        lcd.print("Clave Correcta");
        digitalWrite(LED_GREEN, HIGH); 
        input = Input::Sign_P;
        //delay(2000); // Espera 2 segundos
        
        //reset();
      } else {
        attempts++;
        lcd.clear();
        if(attempts<3){
          digitalWrite(LED_BLUE, HIGH); 
        }
        lcd.print("Clave Incorrecta");
				delay(500);
        reset();
      }
    } else if (key == '*') { // Limpiar entrada
      idx = 0;
      lcd.clear();
      lcd.print("Ingresa clave:");
      lcd.setCursor(0, 1);
    } else { // Almacenar el carácter ingresado
      if (idx < 4) { // Limitar a 4 dígitos
        enteredPassword[idx++] = key;
        lcd.print(key); // Mostrar asterisco por cada carácter ingresado
      }
    }
    digitalWrite(LED_BLUE, LOW);
    // Bloquear después de 3 intentos incorrectos
    if (attempts >= 3) {
      lcd.clear();
      //digitalWrite(LED_RED, HIGH);
      //lcd.print("Sistema Bloqueado");
      input = Input::Sign_S;
      


    }
  }
}
void Alarm_Off(void){
	input = static_cast<Input>(readInput());
	noTone(PIR_SENSOR);
}
void Alarm_On(void){

  for(int i = 200;i <= 800;i++) //frequence loop from 200 to 800
  {
  tone(PIR_SENSOR,i); //turn the buzzer on
  delay(5); //wait for 5 milliseconds
  }
  delay(4000); //wait for 4 seconds on highest frequence
  for(int i = 800;i >= 200;i--)//frequence loop from 800 downto 200
  {
  tone(PIR_SENSOR,i);
  delay(10);
	}
}

void funct_Display(void){
	lcd.clear();
  // print the number of seconds since reset:
  lcd.setCursor(0, 0);
	lcd.print("T: ");
	lcd.print(tempC);

	lcd.setCursor(7, 0);
	lcd.print("H: ");
	lcd.print(Hum);
	
	lcd.setCursor(0, 1);
	lcd.print("IR: ");
	lcd.print(valIR);

	lcd.setCursor(7, 1);
	lcd.print("Hall: ");
	lcd.print(valHall);

}

AsyncTask TaskHumedad(500, true, read_Humedad);
AsyncTask TaskTemperatura(500, true, read_Temperatura);
AsyncTask TaskTimeout(1000, false, funct_Timeout);
AsyncTask TaskTimeout2(1000, false, funct_Timeout2);
AsyncTask TaskTimeout3(1000, false, funct_Timeout3);
AsyncTask TaskDisplay(500, true, funct_Display);
AsyncTask TaskSensor(500, true, read_Sensores);
AsyncTask TaskKeypad(100, true, funct_keypad);
AsyncTask TaskPassword(10, true, read_Password);
AsyncTask TaskAlarm(1000, false, Alarm_On);
AsyncTask TaskAlarmOff(500, false, Alarm_Off);

// Setup the State Machine
void setupStateMachine()
{
	// Add transitions
	stateMachine.AddTransition(INICIO, MONITOREO, []() { return input == Sign_P; });

	stateMachine.AddTransition(INICIO, BLOQUEADO, []() { return input == Sign_S; });
	stateMachine.AddTransition(BLOQUEADO, INICIO, []() { return input == Sign_T; });
	stateMachine.AddTransition(MONITOREO, EVENTOS, []() { return input == Sign_T; });

	stateMachine.AddTransition(EVENTOS, MONITOREO, []() { return input == Sign_T; });
	stateMachine.AddTransition(EVENTOS, ALARMA, []() { return input == Sign_S; });
	stateMachine.AddTransition(MONITOREO, ALARMA, []() { return input == Sign_P; });

	stateMachine.AddTransition(ALARMA, INICIO, []() { return input == Sign_T; });

	// Add actions
	stateMachine.SetOnEntering(INICIO, funct_Inicio);
	stateMachine.SetOnEntering(BLOQUEADO, funct_Bloqueado);
	stateMachine.SetOnEntering(MONITOREO, funct_Monitoreo);
	stateMachine.SetOnEntering(EVENTOS, funct_Eventos);
  stateMachine.SetOnEntering(ALARMA, funct_Alarma);

	stateMachine.SetOnLeaving(INICIO, funct_out_Inicio);
	stateMachine.SetOnLeaving(BLOQUEADO, funct_out_Bloqueado);
	stateMachine.SetOnLeaving(MONITOREO, funt_out_Monitoreo);
	stateMachine.SetOnLeaving(EVENTOS, funct_out_Eventos);
  stateMachine.SetOnLeaving(ALARMA, funct_out_Alarma);
}

void funct_Inicio(void){
	digitalWrite(LED_RED, LOW);
	Serial.println("INICIO");
  lcd.clear();
  lcd.print("INICIO");
  TaskPassword.Start();
}

void funct_out_Inicio(void){
	Serial.println("Leaving INICIO");
  TaskPassword.Stop();
  digitalWrite(LED_GREEN, LOW);
}

void funct_Bloqueado(void){
	Serial.println("BLOQUEADO");
  lcd.clear();
  lcd.print("BLOQUEADO");
	TaskAlarm.Start();
  TaskTimeout.Start();
  digitalWrite(LED_RED, HIGH);
  
}

void funct_out_Bloqueado(void){
	TaskAlarm.Stop();
  TaskTimeout.Stop();
	attempts = 0;
	noTone(PIR_SENSOR);
}

void funct_Monitoreo(void){
	Serial.println("MONITOREO");
	TaskTemperatura.Start();
	TaskHumedad.Start();
	TaskDisplay.Start();
	TaskTimeout2.Start();
}

void funt_out_Monitoreo(void){
	Serial.println("Leaving MONITOREO");
	TaskTemperatura.Stop();
	TaskHumedad.Stop();
	TaskTimeout2.Stop();
}

void funct_Eventos(void){
	Serial.println("EVENTOS");
	TaskSensor.Start();
	TaskTimeout3.Start();
}

void funct_out_Eventos(void){
	Serial.println("Leaving EVENTOS");
	TaskSensor.Stop();
	TaskTimeout3.Stop();
}

void funct_Alarma(void){
	Serial.println("ALARMA");
	TaskDisplay.Stop();
	lcd.clear();
	digitalWrite(LED_RED, HIGH);
	TaskAlarm.Start();
	TaskAlarmOff.Start();
  // print the number of seconds since reset:
  lcd.setCursor(0, 0);
	lcd.print("ALARMA");
}

void funct_out_Alarma(void){
	Serial.println("ALARMA");
  lcd.clear();
	noTone(PIR_SENSOR);
	TaskAlarm.Stop();
	TaskAlarmOff.Stop();
  lcd.print("ALARMA");
  
}


void setup() 
{
	// set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");

	pinMode(PIN_IR, INPUT);
  pinMode(PIN_HALL, INPUT);
	pinMode(PIR_SENSOR, OUTPUT);
	Serial.begin(9600);

	Serial.println("Starting State Machine...");
	setupStateMachine();	
	Serial.println("Start Machine Started");

	dht.begin();
	// Initial state
	stateMachine.SetState(INICIO, false, true);
	TaskKeypad.Start();

	pinMode (LED_GREEN, OUTPUT);
  pinMode (LED_BLUE, OUTPUT);
  pinMode (LED_RED, OUTPUT);


}

void loop() 
{
	
	// Read user input
	input = static_cast<Input>(readInput());
	TaskTemperatura.Update();
	TaskHumedad.Update();
	TaskTimeout.Update();
	TaskTimeout2.Update();
	TaskTimeout3.Update();
	TaskDisplay.Update();
	TaskSensor.Update();
	TaskKeypad.Update();
	TaskPassword.Update();
  TaskAlarm.Update();
	TaskAlarmOff.Update();
	// Update State Machine
	stateMachine.Update();
  input = Input::Unknown;
}

// Auxiliar function that reads the user input
int readInput()
{
	Input currentInput = Input::Unknown;
	if (Serial.available())
	{
		char incomingChar = Serial.read();

		switch (incomingChar)
		{
			case 'P': currentInput = Input::Sign_P; 	break;
			case 'T': currentInput = Input::Sign_T; break;
			case 'S': currentInput = Input::Sign_S; break;
			default: break;
		}
	}

	if(key == '*' ){
		currentInput = Input::Sign_T;
		Serial.println("KEY * PRESSED");
		key = 0;
	}
	if(key == '#'){
		currentInput = Input::Sign_T;
		Serial.println("KEY # PRESSED");
		key = 0;
	}
	return currentInput;
}
