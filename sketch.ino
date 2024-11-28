#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include "DHT.h"
#include <LiquidCrystal.h>
#include <Keypad.h>

/**
 * @file main.cpp
 * @brief Controla el sistema de monitoreo y alarmas con sensores, temperatura y humedad.
 * 
 * Este código utiliza una máquina de estados para manejar el flujo entre varios estados del sistema.
 * Además, interactúa con sensores como DHT, PIR, y un teclado para ingresar contraseñas.
 * 
 * @version 1.0
 * @date 2024-11-28
 */

// Definición de pines
#define LED_GREEN 42      ///< Pin para el LED verde
#define LED_BLUE 44       ///< Pin para el LED azul
#define LED_RED 47        ///< Pin para el LED rojo
#define PIN_IR 48         ///< Pin para el sensor IR
#define PIN_HALL 50       ///< Pin para el sensor Hall
#define PIR_SENSOR 10     ///< Pin para el sensor PIR
#define DHTPIN 46         ///< Pin para el sensor DHT
#define DHTTYPE DHT22     ///< Tipo de sensor DHT (DHT22, AM2302)
#define analogPin A0      ///< Pin para el termistor
#define beta 3950         ///< Beta del termistor

/// @brief Macro para depuración de la consola serial
#define DEBUG(a) Serial.print(millis()); Serial.print(": "); Serial.println(a);

// Variables globales
uint64_t value = 0;             ///< Valor para el control de la máquina de estados
char password[5];               ///< Contraseña ingresada por el usuario
unsigned char idx = 0;          ///< Índice para almacenar la contraseña
char correctPassword[] = "1234"; ///< Contraseña correcta
char enteredPassword[5];         ///< Array para almacenar la contraseña ingresada
int attempts = 0;               ///< Contador de intentos fallidos
float Hum;

/// @brief Definición del teclado matricial de 4x4
const byte ROWS = 4; ///< Número de filas en el teclado
const byte COLS = 4; ///< Número de columnas en el teclado
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {24, 26, 28, 30}; ///< Pines para las filas del teclado
byte colPins[COLS] = {32, 34, 36};     ///< Pines para las columnas del teclado
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS ); ///< Objeto teclado

/// @brief Inicialización de la pantalla LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; ///< Pines de la pantalla LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); ///< Objeto para controlar la pantalla LCD

/// @brief Inicialización del sensor DHT
DHT dht(DHTPIN, DHTTYPE); ///< Objeto DHT para leer temperatura y humedad
float tempC = 0.0;         ///< Variable para almacenar la temperatura en °C

/// @brief Alias para los estados de la máquina de estados
enum State
{
    INICIO = 0,     ///< Estado inicial
    BLOQUEADO = 1,  ///< Estado bloqueado (después de intentos fallidos)
    MONITOREO = 2,  ///< Estado de monitoreo de sensores
    EVENTOS = 3,    ///< Estado de eventos (como el sensor PIR)
    ALARMA = 4      ///< Estado de alarma activa
};

/// @brief Alias para las entradas del sistema
enum Input
{
    Sign_T = 0,     ///< Entrada de tiempo (reset o espera)
    Sign_P = 1,     ///< Entrada de contraseña correcta
    Sign_S = 2,     ///< Entrada de sensor (IR o Hall)
    Unknown = 3     ///< Entrada desconocida
};

// Creación de la máquina de estados
StateMachine stateMachine(5, 8);

// Variable para almacenar la última entrada del usuario
Input input;

/**
 * @brief Lee la temperatura del sensor DHT y la temperatura analógica del termistor.
 * 
 * Esta función obtiene la temperatura en dos formas: digital (DHT) y analógica (termistor).
 * Si la temperatura digital supera los 28°C, se registra una entrada de tipo Sign_P.
 */
void read_Temperatura(void){
    Serial.println("TEMP_DIGITAL");
    // Leer temperatura desde el sensor DHT
    tempC = dht.readTemperature();
    Serial.print("TEMP: ");
    Serial.println(tempC);
    Serial.println("TEMP_ANALOG");
    
    // Leer el valor del termistor
    long a = analogRead(analogPin);
    
    // Calcular la temperatura a partir del valor analógico
    tempC = 1 / (log(1 / (1023.0 / a - 1)) / beta + 1.0 / 298.15) - 273.15;
    Serial.print("TEMP: ");
    Serial.println(tempC);
    
    // Si la temperatura supera los 28°C, actualizar la entrada a Sign_P
    if(tempC > 28.0){
        input = Input::Sign_P;
    }
}

/**
 * @brief Lee la humedad desde el sensor DHT.
 */
void read_Humedad(void){
    Hum = dht.readHumidity(); 
}

/**
 * @brief Función de tiempo de espera para un evento.
 * 
 * Espera 7 segundos antes de restablecer la entrada a Sign_T.
 */
void funct_Timeout(void){
    delay(7000);
    input = Input::Sign_T; 
}

/**
 * @brief Función de tiempo de espera para otro evento.
 * 
 * Espera 5 segundos antes de restablecer la entrada a Sign_T.
 */
void funct_Timeout2(void){
    delay(5000);
    input = Input::Sign_T; 
}

/**
 * @brief Función de tiempo de espera para un evento.
 * 
 * Esta función espera 3 segundos antes de restablecer la entrada a Sign_T.
 */
void funct_Timeout3(void){
    delay(3000);
    input = Input::Sign_T; 
}

/**
 * @brief Variables de estado de los sensores IR y Hall.
 * 
 * Estas variables almacenan el valor leído desde los sensores de movimiento IR y Hall.
 */
unsigned char valIR = LOW;    ///< Estado del sensor IR
unsigned char valHall = LOW;  ///< Estado del sensor Hall

/**
 * @brief Lee los valores de los sensores IR y Hall.
 * 
 * Esta función lee el estado de los sensores de movimiento y detecta si se ha activado
 * el sensor IR o Hall. Si alguno de los sensores detecta un evento, actualiza la entrada
 * a Sign_S.
 */
void read_Sensores(void){
    valIR = digitalRead(PIN_IR);      ///< Lee el valor del sensor IR
    valHall = digitalRead(PIN_HALL);  ///< Lee el valor del sensor Hall

    if((valIR == HIGH) || (valHall == HIGH)){
        input = Input::Sign_S; ///< Si se detecta movimiento o luz, se activa Sign_S
    }

    // Imprime el estado de los sensores IR y Hall en el monitor serial
    if(valIR == LOW){
        Serial.print("IR: ");
        Serial.println("No motion");
    } else {
        Serial.print("IR: ");
        Serial.println("Motion");
    }

    if(valHall == LOW){
        Serial.print("HALL: ");
        Serial.println("Light");
    } else {
        Serial.print("HALL: ");
        Serial.println("Dark");
    }
}

/**
 * @brief Función para leer la tecla presionada en el teclado matricial.
 * 
 * Esta función lee la tecla presionada y la imprime en el monitor serial.
 */
char key = 0; ///< Variable para almacenar la tecla presionada
void funct_keypad(void){
    key = keypad.getKey(); ///< Lee la tecla del teclado matricial
  
    if (key){
        Serial.println(key); ///< Imprime la tecla presionada
    }
}

/**
 * @brief Verifica si la contraseña ingresada es correcta.
 * 
 * Compara la contraseña ingresada con la contraseña correcta predefinida.
 * 
 * @param password La contraseña ingresada por el usuario.
 * @return True si la contraseña es correcta, false en caso contrario.
 */
bool checkPassword(char *password) {
    return strcmp(password, correctPassword) == 0; ///< Compara la contraseña ingresada con la correcta
}

/**
 * @brief Restablece el estado de la pantalla LCD y prepara el sistema para ingresar la contraseña.
 * 
 * Esta función reinicia el índice de la contraseña, limpia la pantalla LCD y muestra el mensaje
 * solicitando la clave.
 */
void reset() {
    idx = 0;           ///< Reinicia el índice de la contraseña
    lcd.clear();       ///< Limpia la pantalla LCD
    lcd.print("Ingresa clave:"); ///< Muestra el mensaje de solicitud de clave
    lcd.setCursor(0, 1); ///< Mueve el cursor a la segunda línea de la LCD
}

/**
 * @brief Lee y procesa la contraseña ingresada desde el teclado.
 * 
 * Esta función lee las teclas presionadas en el teclado y las almacena. Cuando el usuario presiona
 * el símbolo '#' verifica la contraseña. Si la contraseña es correcta, activa el LED verde y
 * pasa al estado Sign_P. Si es incorrecta, muestra un mensaje y restablece el sistema. Después de
 * 3 intentos fallidos, bloquea el sistema.
 */
void read_Password(void){
    char key = keypad.getKey(); ///< Lee la tecla del teclado
 
    if (key) {
        if (key == '#') { ///< Si el usuario presiona '#', verifica la contraseña
            enteredPassword[idx] = '\0';  ///< Termina la cadena de la contraseña ingresada
            if (checkPassword(enteredPassword)) { ///< Si la contraseña es correcta
                lcd.clear();
                lcd.print("Clave Correcta"); ///< Muestra el mensaje de clave correcta
                digitalWrite(LED_GREEN, HIGH); ///< Enciende el LED verde
                input = Input::Sign_P; ///< Cambia al estado Sign_P
                //delay(2000); // Espera 2 segundos antes de continuar
            } else { ///< Si la contraseña es incorrecta
                attempts++; ///< Incrementa el contador de intentos
                lcd.clear();
                if(attempts < 3) {
                    digitalWrite(LED_BLUE, HIGH); ///< Enciende el LED azul
                }
                lcd.print("Clave Incorrecta"); ///< Muestra el mensaje de clave incorrecta
                delay(500); ///< Espera medio segundo antes de continuar
                reset(); ///< Reinicia el proceso de ingreso de contraseña
            }
        } else if (key == '*') { ///< Si el usuario presiona '*', limpia la entrada
            idx = 0; ///< Reinicia el índice de la contraseña
            lcd.clear();
            lcd.print("Ingresa clave:"); ///< Muestra el mensaje de solicitud de clave
            lcd.setCursor(0, 1); ///< Mueve el cursor a la segunda línea de la LCD
        } else { ///< Si el usuario ingresa una tecla válida
            if (idx < 4) { ///< Limita la contraseña a 4 dígitos
                enteredPassword[idx++] = key; ///< Almacena el carácter ingresado
                lcd.print(key); ///< Muestra un asterisco por cada carácter ingresado
            }
        }
        digitalWrite(LED_BLUE, LOW); ///< Apaga el LED azul
        // Bloquear después de 3 intentos incorrectos
        if (attempts >= 3) {
            lcd.clear();
            //digitalWrite(LED_RED, HIGH); ///< Enciende el LED rojo en caso de bloqueo
            //lcd.print("Sistema Bloqueado"); ///< Muestra el mensaje de bloqueo
            input = Input::Sign_S; ///< Cambia al estado Sign_S (bloqueado)
        }
    }
}

/**
 * @brief Apaga la alarma.
 * 
 * Esta función detiene el tono del sensor PIR y cambia el estado de entrada.
 */
void Alarm_Off(void){
    input = static_cast<Input>(readInput()); ///< Lee la entrada y cambia el estado
    noTone(PIR_SENSOR); ///< Detiene el tono del buzzer (alarma)
}

/**
 * @brief Enciende la alarma.
 * 
 * Esta función genera un sonido de alarma variando la frecuencia del buzzer
 * desde 200 Hz hasta 800 Hz y luego vuelve a 200 Hz.
 */
void Alarm_On(void){
    // Genera un tono de alarma con frecuencia variable
    for(int i = 200; i <= 800; i++) { 
        tone(PIR_SENSOR, i); ///< Activa el buzzer con la frecuencia actual
        delay(5); ///< Espera 5 milisegundos antes de cambiar la frecuencia
    }
    delay(4000); ///< Espera 4 segundos en la frecuencia más alta
    // Regresa la frecuencia de 800 Hz a 200 Hz
    for(int i = 800; i >= 200; i--) { 
        tone(PIR_SENSOR, i); ///< Activa el buzzer con la frecuencia decreciente
        delay(10); ///< Espera 10 milisegundos entre cambios de frecuencia
    }
}

/**
 * @brief Muestra los valores de temperatura, humedad, IR y Hall en la pantalla LCD.
 * 
 * Esta función limpia la pantalla LCD y muestra los valores actuales de la
 * temperatura, humedad, estado del sensor IR y sensor Hall en la interfaz de usuario.
 */
void funct_Display(void){
    lcd.clear(); ///< Limpia la pantalla LCD
    // Muestra la temperatura en la primera línea
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print(tempC);

    // Muestra la humedad en la primera línea
    lcd.setCursor(7, 0);
    lcd.print("H: ");
    lcd.print(Hum);

    // Muestra el estado del sensor IR en la segunda línea
    lcd.setCursor(0, 1);
    lcd.print("IR: ");
    lcd.print(valIR);

    // Muestra el estado del sensor Hall en la segunda línea
    lcd.setCursor(7, 1);
    lcd.print("Hall: ");
    lcd.print(valHall);
}

// Tareas asíncronas para leer sensores y manejar el sistema
AsyncTask TaskHumedad(500, true, read_Humedad);  ///< Tarea para leer humedad cada 500 ms
AsyncTask TaskTemperatura(500, true, read_Temperatura);  ///< Tarea para leer temperatura cada 500 ms
AsyncTask TaskTimeout(1000, false, funct_Timeout);  ///< Tarea para manejar el tiempo de espera
AsyncTask TaskTimeout2(1000, false, funct_Timeout2);  ///< Tarea para manejar otro tiempo de espera
AsyncTask TaskTimeout3(1000, false, funct_Timeout3);  ///< Tarea para manejar otro tiempo de espera
AsyncTask TaskDisplay(500, true, funct_Display);  ///< Tarea para actualizar la pantalla LCD cada 500 ms
AsyncTask TaskSensor(500, true, read_Sensores);  ///< Tarea para leer los sensores cada 500 ms
AsyncTask TaskKeypad(100, true, funct_keypad);  ///< Tarea para leer el teclado cada 100 ms
AsyncTask TaskPassword(10, true, read_Password);  ///< Tarea para verificar la contraseña cada 10 ms
AsyncTask TaskAlarm(1000, false, Alarm_On);  ///< Tarea para encender la alarma cada 1000 ms
AsyncTask TaskAlarmOff(500, false, Alarm_Off);  ///< Tarea para apagar la alarma cada 500 ms

/**
 * @brief Configura la máquina de estados.
 * 
 * Esta función configura las transiciones y las acciones que deben ejecutarse
 * cuando se entra o sale de un estado en la máquina de estados.
 */
void setupStateMachine()
{
    // Añade las transiciones entre estados
    stateMachine.AddTransition(INICIO, MONITOREO, []() { return input == Sign_P; }); ///< De INICIO a MONITOREO si la entrada es Sign_P
    stateMachine.AddTransition(INICIO, BLOQUEADO, []() { return input == Sign_S; }); ///< De INICIO a BLOQUEADO si la entrada es Sign_S
    stateMachine.AddTransition(BLOQUEADO, INICIO, []() { return input == Sign_T; }); ///< De BLOQUEADO a INICIO si la entrada es Sign_T
    stateMachine.AddTransition(MONITOREO, EVENTOS, []() { return input == Sign_T; }); ///< De MONITOREO a EVENTOS si la entrada es Sign_T
    stateMachine.AddTransition(EVENTOS, MONITOREO, []() { return input == Sign_T; }); ///< De EVENTOS a MONITOREO si la entrada es Sign_T
    stateMachine.AddTransition(EVENTOS, ALARMA, []() { return input == Sign_S; }); ///< De EVENTOS a ALARMA si la entrada es Sign_S
    stateMachine.AddTransition(MONITOREO, ALARMA, []() { return input == Sign_P; }); ///< De MONITOREO a ALARMA si la entrada es Sign_P
    stateMachine.AddTransition(ALARMA, INICIO, []() { return input == Sign_T; }); ///< De ALARMA a INICIO si la entrada es Sign_T

    // Configura las acciones a realizar al entrar en cada estado
    stateMachine.SetOnEntering(INICIO, funct_Inicio);  ///< Acción al entrar en el estado INICIO
    stateMachine.SetOnEntering(BLOQUEADO, funct_Bloqueado);  ///< Acción al entrar en el estado BLOQUEADO
    stateMachine.SetOnEntering(MONITOREO, funct_Monitoreo);  ///< Acción al entrar en el estado MONITOREO
    stateMachine.SetOnEntering(EVENTOS, funct_Eventos);  ///< Acción al entrar en el estado EVENTOS
    stateMachine.SetOnEntering(ALARMA, funct_Alarma);  ///< Acción al entrar en el estado ALARMA

    // Configura las acciones a realizar al salir de cada estado
    stateMachine.SetOnLeaving(INICIO, funct_out_Inicio);  ///< Acción al salir del estado INICIO
    stateMachine.SetOnLeaving(BLOQUEADO, funct_out_Bloqueado);  ///< Acción al salir del estado BLOQUEADO
    stateMachine.SetOnLeaving(MONITOREO, funt_out_Monitoreo);  ///< Acción al salir del estado MONITOREO
    stateMachine.SetOnLeaving(EVENTOS, funct_out_Eventos);  ///< Acción al salir del estado EVENTOS
    stateMachine.SetOnLeaving(ALARMA, funct_out_Alarma);  ///< Acción al salir del estado ALARMA
}


/**
 * @brief Acción al entrar en el estado INICIO.
 * 
 * Esta función se ejecuta cuando la máquina de estados entra en el estado INICIO.
 * Apaga el LED rojo, imprime un mensaje en el LCD y comienza la tarea de contraseña.
 */
void funct_Inicio(void){
    digitalWrite(LED_RED, LOW); ///< Apaga el LED rojo
    Serial.println("INICIO"); ///< Imprime mensaje en el monitor serial
    lcd.clear(); ///< Limpia la pantalla LCD
    lcd.print("INICIO"); ///< Muestra "INICIO" en el LCD
    TaskPassword.Start(); ///< Inicia la tarea de verificación de contraseña
}

/**
 * @brief Acción al salir del estado INICIO.
 * 
 * Esta función se ejecuta cuando la máquina de estados sale del estado INICIO.
 * Imprime un mensaje de salida y detiene la tarea de contraseña.
 */
void funct_out_Inicio(void){
    Serial.println("Leaving INICIO"); ///< Imprime mensaje en el monitor serial
    TaskPassword.Stop(); ///< Detiene la tarea de verificación de contraseña
    digitalWrite(LED_GREEN, LOW); ///< Apaga el LED verde
}

/**
 * @brief Acción al entrar en el estado BLOQUEADO.
 * 
 * Esta función se ejecuta cuando la máquina de estados entra en el estado BLOQUEADO.
 * Muestra "BLOQUEADO" en el LCD, inicia la alarma, el temporizador y enciende el LED rojo.
 */
void funct_Bloqueado(void){
    Serial.println("BLOQUEADO"); ///< Imprime mensaje en el monitor serial
    lcd.clear(); ///< Limpia la pantalla LCD
    lcd.print("BLOQUEADO"); ///< Muestra "BLOQUEADO" en el LCD
    TaskAlarm.Start(); ///< Inicia la tarea de la alarma
    TaskTimeout.Start(); ///< Inicia la tarea de temporizador
    digitalWrite(LED_RED, HIGH); ///< Enciende el LED rojo
}

/**
 * @brief Acción al salir del estado BLOQUEADO.
 * 
 * Esta función se ejecuta cuando la máquina de estados sale del estado BLOQUEADO.
 * Detiene las tareas de alarma y temporizador, reinicia el contador de intentos y apaga el buzzer.
 */
void funct_out_Bloqueado(void){
    TaskAlarm.Stop(); ///< Detiene la tarea de la alarma
    TaskTimeout.Stop(); ///< Detiene la tarea de temporizador
    attempts = 0; ///< Reinicia el contador de intentos
    noTone(PIR_SENSOR); ///< Apaga el buzzer
}

/**
 * @brief Acción al entrar en el estado MONITOREO.
 * 
 * Esta función se ejecuta cuando la máquina de estados entra en el estado MONITOREO.
 * Inicia las tareas de temperatura, humedad, pantalla LCD y temporizador.
 */
void funct_Monitoreo(void){
    Serial.println("MONITOREO"); ///< Imprime mensaje en el monitor serial
    TaskTemperatura.Start(); ///< Inicia la tarea de lectura de temperatura
    TaskHumedad.Start(); ///< Inicia la tarea de lectura de humedad
    TaskDisplay.Start(); ///< Inicia la tarea de actualización de la pantalla LCD
    TaskTimeout2.Start(); ///< Inicia la tarea de temporizador
}

/**
 * @brief Acción al salir del estado MONITOREO.
 * 
 * Esta función se ejecuta cuando la máquina de estados sale del estado MONITOREO.
 * Detiene las tareas de temperatura, humedad y temporizador.
 */
void funt_out_Monitoreo(void){
    Serial.println("Leaving MONITOREO"); ///< Imprime mensaje en el monitor serial
    TaskTemperatura.Stop(); ///< Detiene la tarea de lectura de temperatura
    TaskHumedad.Stop(); ///< Detiene la tarea de lectura de humedad
    TaskTimeout2.Stop(); ///< Detiene la tarea de temporizador
}

/**
 * @brief Acción al entrar en el estado EVENTOS.
 * 
 * Esta función se ejecuta cuando la máquina de estados entra en el estado EVENTOS.
 * Inicia las tareas de sensores y temporizador.
 */
void funct_Eventos(void){
    Serial.println("EVENTOS"); ///< Imprime mensaje en el monitor serial
    TaskSensor.Start(); ///< Inicia la tarea de lectura de sensores
    TaskTimeout3.Start(); ///< Inicia la tarea de temporizador
}

/**
 * @brief Acción al salir del estado EVENTOS.
 * 
 * Esta función se ejecuta cuando la máquina de estados sale del estado EVENTOS.
 * Detiene las tareas de sensores y temporizador.
 */
void funct_out_Eventos(void){
    Serial.println("Leaving EVENTOS"); ///< Imprime mensaje en el monitor serial
    TaskSensor.Stop(); ///< Detiene la tarea de lectura de sensores
    TaskTimeout3.Stop(); ///< Detiene la tarea de temporizador
}

/**
 * @brief Acción al entrar en el estado ALARMA.
 * 
 * Esta función se ejecuta cuando la máquina de estados entra en el estado ALARMA.
 * Detiene la tarea de pantalla LCD, muestra "ALARMA" en la LCD, enciende el LED rojo y empieza la alarma.
 */
void funct_Alarma(void){
    Serial.println("ALARMA"); ///< Imprime mensaje en el monitor serial
    TaskDisplay.Stop(); ///< Detiene la tarea de actualización de la pantalla LCD
    lcd.clear(); ///< Limpia la pantalla LCD
    digitalWrite(LED_RED, HIGH); ///< Enciende el LED rojo
    TaskAlarm.Start(); ///< Inicia la tarea de la alarma
    TaskAlarmOff.Start(); ///< Inicia la tarea para apagar la alarma
    lcd.setCursor(0, 0); ///< Establece el cursor en la primera línea
    lcd.print("ALARMA"); ///< Muestra "ALARMA" en la pantalla LCD
}

/**
 * @brief Acción al salir del estado ALARMA.
 * 
 * Esta función se ejecuta cuando la máquina de estados sale del estado ALARMA.
 * Detiene las tareas de alarma, apaga el buzzer, limpia la pantalla y muestra "ALARMA" en la LCD.
 */
void funct_out_Alarma(void){
    Serial.println("ALARMA"); ///< Imprime mensaje en el monitor serial
    lcd.clear(); ///< Limpia la pantalla LCD
    noTone(PIR_SENSOR); ///< Apaga el buzzer
    TaskAlarm.Stop(); ///< Detiene la tarea de la alarma
    TaskAlarmOff.Stop(); ///< Detiene la tarea para apagar la alarma
    lcd.print("ALARMA"); ///< Muestra "ALARMA" en la pantalla LCD
}


/**
 * @brief Configuración inicial del sistema.
 * 
 * Esta función se ejecuta una sola vez al inicio del programa. Configura la pantalla LCD, los pines de entrada/salida,
 * la comunicación serial, las tareas y la máquina de estados.
 */
void setup() 
{
    // Configura el número de columnas y filas del LCD
    lcd.begin(16, 2); ///< Inicializa la pantalla LCD con 16 columnas y 2 filas
    lcd.print("hello, world!"); ///< Muestra "hello, world!" en el LCD

    // Configura los pines de entrada y salida
    pinMode(PIN_IR, INPUT); ///< Configura el PIN_IR como entrada
    pinMode(PIN_HALL, INPUT); ///< Configura el PIN_HALL como entrada
    pinMode(PIR_SENSOR, OUTPUT); ///< Configura el PIR_SENSOR como salida
    Serial.begin(9600); ///< Inicializa la comunicación serial a 9600 baudios

    Serial.println("Starting State Machine..."); ///< Imprime en el monitor serial
    setupStateMachine(); ///< Configura la máquina de estados
    Serial.println("Start Machine Started"); ///< Imprime en el monitor serial

    dht.begin(); ///< Inicia el sensor DHT para leer temperatura y humedad

    // Establece el estado inicial de la máquina de estados
    stateMachine.SetState(INICIO, false, true); ///< Establece el estado INICIO como el inicial

    // Inicia la tarea de lectura de teclado
    TaskKeypad.Start();

    // Configura los pines de salida para los LEDs
    pinMode (LED_GREEN, OUTPUT); ///< Configura el LED verde como salida
    pinMode (LED_BLUE, OUTPUT); ///< Configura el LED azul como salida
    pinMode (LED_RED, OUTPUT); ///< Configura el LED rojo como salida
}

/**
 * @brief Función principal que se ejecuta en un bucle.
 * 
 * Esta función se ejecuta repetidamente durante el ciclo de vida del programa. Actualiza las tareas y la máquina de estados.
 */
void loop() 
{
    // Lee la entrada del usuario
    input = static_cast<Input>(readInput()); ///< Lee la entrada del usuario y la asigna a 'input'

    // Actualiza las tareas
    TaskTemperatura.Update(); ///< Actualiza la tarea de lectura de temperatura
    TaskHumedad.Update(); ///< Actualiza la tarea de lectura de humedad
    TaskTimeout.Update(); ///< Actualiza la tarea de temporizador
    TaskTimeout2.Update(); ///< Actualiza la tarea de temporizador 2
    TaskTimeout3.Update(); ///< Actualiza la tarea de temporizador 3
    TaskDisplay.Update(); ///< Actualiza la tarea de actualización de pantalla LCD
    TaskSensor.Update(); ///< Actualiza la tarea de lectura de sensores
    TaskKeypad.Update(); ///< Actualiza la tarea de lectura de teclado
    TaskPassword.Update(); ///< Actualiza la tarea de verificación de contraseña
    TaskAlarm.Update(); ///< Actualiza la tarea de la alarma
    TaskAlarmOff.Update(); ///< Actualiza la tarea para apagar la alarma

    // Actualiza la máquina de estados
    stateMachine.Update();

    // Restablece la entrada a desconocida
    input = Input::Unknown; ///< Establece 'input' como desconocido al final del ciclo
}

/**
 * @brief Función auxiliar que lee la entrada del usuario desde la consola o el teclado.
 * 
 * Esta función lee el carácter entrante desde la comunicación serial o el teclado, y lo mapea a los valores de entrada correspondientes.
 * @return Input El valor de la entrada leída.
 */
int readInput()
{
    Input currentInput = Input::Unknown; ///< Inicializa la variable de entrada con valor desconocido

    // Si hay datos disponibles en el puerto serial, los lee
    if (Serial.available())
    {
        char incomingChar = Serial.read(); ///< Lee el carácter entrante desde el puerto serial

        // Mapea el carácter recibido a los valores de entrada correspondientes
        switch (incomingChar)
        {
            case 'P': currentInput = Input::Sign_P; break; ///< Asigna la entrada Sign_P si recibe 'P'
            case 'T': currentInput = Input::Sign_T; break; ///< Asigna la entrada Sign_T si recibe 'T'
            case 'S': currentInput = Input::Sign_S; break; ///< Asigna la entrada Sign_S si recibe 'S'
            default: break; ///< No hace nada si el carácter es diferente
        }
    }

    // Verifica las teclas del teclado y actualiza la entrada
    if (key == '*' ){
        currentInput = Input::Sign_T; ///< Asigna la entrada Sign_T si se presiona '*'
        Serial.println("KEY * PRESSED"); ///< Imprime en el monitor serial
        key = 0; ///< Restablece la tecla
    }
    if (key == '#'){
        currentInput = Input::Sign_T; ///< Asigna la entrada Sign_T si se presiona '#'
        Serial.println("KEY # PRESSED"); ///< Imprime en el monitor serial
        key = 0; ///< Restablece la tecla
    }

    return currentInput; ///< Retorna el valor de la entrada
}