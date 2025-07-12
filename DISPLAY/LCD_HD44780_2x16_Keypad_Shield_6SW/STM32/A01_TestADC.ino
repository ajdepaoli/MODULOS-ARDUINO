// ########### DATOS INICIALES ###########
// Nombre: LCD_Keypad_Shield+I2C_A01v
// Programa para: TEST de MODULOS
// Módulo: TEST de "STM32F103C8T6 BluePill" + "I2C" + "LCD Keypad Shield"
// Creador y programador: Alejandro Jorge Depaoli
// Ayudante: IA COPILOT
// Fecha de Inicialización: 06/07/2025
// Versión: A01
// Ultima Revisión: 11/07/2025
//
// ### PARAMETROS Básicos para "ARDUINO-IDE" ###
// Placa: "Generic STM32F1 series"								>	SELECIONAR este ITEM
// Puerto: "COMx"												[según sea el que se use]
// Reload Board Data											>	¡ ¡ ¡ O J O ! ! ! Desconfigura todo
// Obtener información de la placa
// ------------
// Debug symbols and core logs: "None"							>	Por ahora sin DEBUG
// Optimize: "Faster (-02) with LT"								>	Termino Medio para la Compilación 
// Board part number: "BluePill F103CB (or C8 with 128k)"		> 	SELECIONAR este ITEM ¡ ¡ ¡ O J O ! ! ! el AZUL
// C Runtime Library: "Newlib Nano (default)"					>	Sin uso de la CONSOLA
// Upload method: "STM32CubeProgrammer (SWD)"					>	SELECIONAR este ITEM para ST-LINK
// USB support (if available): "None"							>	Conexión por ST-LINK
// U(S)ART support: "Disabled (no Serial support)"				>	Conexión por ST-LINK y Sin uso de la CONSOLA
// USB speed (if available): "Low/Full Speed"					>	Conexión por ST-LINK, sin importancia
//
//
//
//
//  Ж Ж Ж Ж Ж Ж Ж CONFIGURACIÓN DE PARÁMETROS INICIALES  Ж Ж Ж Ж Ж Ж Ж
// Declaración e inicialización de bibliotecas, variables, E/S, funciones y otros recursos
//
//
// ════════════ I N D I C E ════════════
// IA00 → BIBLIOTECAS
//   IA01 → Tiempo y Sistema (delay, timers, FreeRTOS, bajo consumo)
//   IA02 → Memoria / EEPROM
//   IA03 → Funciones Estándar (stdlib, string, math)
//   IA04 → Comunicaciones (I²C, SPI, Serial, etc.)
//   IA05 → Periféricos y Pantallas (LCD, OLED, Servo)
//
// IB00 → DEFINICIÓN de los PINES
//  IB01 → Entradas digitales
//  IB02 → Salidas digitales
//  IB03 → Entradas analógicas (ADC_IN)
//  IB04 → Comparación analógica por software
//  IB05 → Salidas analógicas (DAC emulado por PWM)
//  IB06 → Salidas PWM
//  IB07 → Pines de Comunicación (UART, SPI, I²C…)
//  IB08 → Interrupciones externas (EXTI)
//  IB09 → Timers por hardware
//  IB10 → Bootloader y Programación
//  IB11 → Alimentación
//  IB12 → Referencias ADC internas (VREFINT, Temp)
//
// IC00 → VARIABLES
//    IC01 → OBJETO GLOBALES
//    IC02 → ESCALARES GLOBALES
//    IC03 → LISTAS
//    IC04 → ESCALARES LOCALES
//
// ID00 → CONSTANTES Y MACROS
//    ID01 → Constantes literales (const, #define)
//    ID02 → Enumeraciones (enum)
//    ID03 → Macros complejas
//
// IE00 → SUBRUTINAS
//   IE01 → EEPROM (guardar y recuperar)
//   IE02 → Uso Único (setup o llamada 1 vez)
//   IE03 → 2º Plano Permanente
//   IE04 → 2º Plano No Permanente (Timers, ISR)
//   IE05 → Subrutinas de Uso Común
//
// IF00 → setup()
//
// IG00 → loop()
//
// IH00 → DOCUMENTACIÓN Y NOTAS
//
//
// ─────────────────────────────────────
// NOTAS:
//	- Podés reducir o eliminar partes del índice una vez armado el SKETCH final.
//	- Las etiquetas tipo IA01 a IH00 ayudan a navegar el archivo desde editores simples.
//	- "//```Cpp" y al final "```", te ayudan para trabajar con la IA.
//	- En este SKETCH dejé el Indice completo, por si te sirve de ayuda para el futuro
//	- Este INDICE puede ir cambiando, voy a dejar actualizaciones a medida que vaya creciendo mi conocimiento.
//	- NO se ha usado la función bloqueante 'delay()', todos los tiempos se trabajan con millis(),
//		puede que se un poco más compleja, pero usar 'delay()' puede crear atascos tiempos.
//
//```Cpp
//
// "IA00"
// Ж Ж Ж Ж Ж --- BIBLIOTECAS --- Ж Ж Ж Ж Ж
//
// "IA04"
// Ж Ж Ж --- COMUNICACIONES --- Ж Ж Ж
#include <Wire.h>                // I²C: sensores, RTC, expanders
//
// "IA05"
// Ж Ж Ж --- PERIFÉRICOS Y DISPLAYS --- Ж Ж Ж
#include <LiquidCrystal_I2C.h>       // LCD estándar (HD44780) -- Autor: Frank de Brabander
//
//
// "IB00"
// Ж Ж Ж Ж Ж Ж Ж --- DEFINICIÓN de los PINES (ENTRADAS y SALIDAS)--- Ж Ж Ж Ж Ж Ж Ж
//
// "IB02"
// Ж Ж Ж SALIDAS DIGITALES Low/High o 0V/3.3V Ж Ж Ж
const int ledPinPB12 = PB12;              // LED en pin B12 (valor = -32,768 a 32,767)
//
// "IB03"
// Ж Ж Ж ENTRADAS ANALOGICAS Low/High o 0V/3.3V Ж Ж Ж
const int pinTeclado = PA0; // Teclado ADC del "Shield LCD Keypad"  (valor = -32,768 a 32,767)
// 
// "IC00"
// Ж Ж Ж Ж Ж ------- VARIABLES ------- Ж Ж Ж Ж Ж
//
// "IC01"
// Ж Ж Ж ----- OBJETO GLOBALES ----- Ж Ж Ж
// LCD 1602 o HD44780
LiquidCrystal_I2C lcd(0x27, 16, 2);  // DIR del BUS, Cantidad de COLUMNAS, Cantidad de LINEAS
//
// "IC02"
// Ж Ж Ж ----- ESCALARES GLOBALES ----- Ж Ж Ж
// unsigned long = 0 a 4,294,967,295; sin 'unsigned log' = -32,768 a 32,767
// Variables del LED de Control PB12
unsigned long ultimoTiempoPB12 = 0;				// Contador para el tiempo y depende de 'millis()'
const unsigned long tiempoParpadeoB12 = 500;	// tiempo de parpadeo del Led; 0.5 segundos
bool ledStatePB12 = false;						// Estado inicial del Led = apagado (valor = false/true -- bits = 1)
// Variables del Teclado analógico en A0
const int teclaPinA0 = PA0;				// Valor ingresado en PA0
int valorPA0 = 0;						// Valor leído del ADC y puesta a Cero
int umbralTeclasPA0 = 900;				// Valor del umbral para detectar una tecla cualquiera (<3V)
bool mostrandoPA0 = false;
// Variables para el LCD de la LINEA 
unsigned long tiempoInicioLínea1 = 0;		// Tiempo Inicial de LINEA1, depende de 'millis()'
const unsigned long tiempoLínea1 = 3000;	// Tiempo de retardo de la LINEA 1; 3 segundos
//
// "IF00"
// Д Д Д Д Д Д Д ------- setup() ------- Д Д Д Д Д Д 
void setup() {
// Д Д Д Д Д ----- CONFIGURACION INICIAL ----- Д Д Д Д Д
// Д Д Д Configuración de los Pines como SALIDAS Д Д Д
  Wire.begin(PB7, PB6);					// Activo salida "I2C", SDA = PB7, SCL = PB6
  pinMode(ledPinPB12, OUTPUT);				// LED de diagnostico
  digitalWrite(ledPinPB12, LOW);			// LED apagado
  pinMode(teclaPinA0, INPUT);				// Entrada del TECLADO.
// Д Д Д Configuración del LCD Д Д Д
  lcd.init();							// Inicia el LCD
  lcd.noBacklight();					// Enciende el LCD para: "I2C" + "LCD Keypad Shield"
//  lcd.backlight();					// Enciende el LCD para: "I2C" + "LCD"
// Escritura en el LCD.
  lcd.clear();							// Borra el LCD
  lcd.setCursor(0, 0);					// Celda 0, Línea 0 -- Parámetro para iniciar la escritura.
  lcd.print("  Hola MUNDO !  ");		// Escritura en LCD
  lcd.setCursor(0, 1);					// Celda 0, Línea 1 -- Parámetro para iniciar la escritura.
  lcd.print("presionar SWITCH ");		// Escritura en LCD
}
//
// "IG00"
// Ē Ē Ē Ē Ē Ē Ē ------- loop() ------- Ǝ Ǝ Ǝ Ǝ Ǝ Ǝ Ǝ 
void loop() {
//
// SVL01 "Parpadeo del LED"
  if (millis() - ultimoTiempoPB12 >= tiempoParpadeoB12) {
    ultimoTiempoPB12 = millis();
    ledStatePB12 = !ledStatePB12;							// Invierto el estado de la variable PB12 ---- ('!' invertir)
    digitalWrite(ledPinPB12, ledStatePB12 ? HIGH : LOW);
  } // "FIN" de SVL01
//
// SVL02 "Lectura del teclado y muestra el valor ADC"
  if (!mostrandoPA0) {									// Si es True, Salto a 'SLV02-A' ---- ('!' invertir)
    valorPA0 = analogRead(teclaPinA0);					// Valor de ADC según 'PA0'; y Salto a 'SVL02-C'
    // SVL02-A >> Mostrar "SW=ON - ADC=", SOLO una vez <<
    if (valorPA0 < umbralTeclasPA0 && !mostrandoPA0) {	// Si es 'False' Salto a 'SVL02-B'
      mostrandoPA0 = true;								// Activa: Mostrando "SW 'OK'-ADC="
      tiempoInicioLínea1 = millis();					// Tiempo Inicial del ciclo 'Línea1'
      lcd.setCursor(0, 1);								// Celda 0, Línea 1 -- Parámetro para iniciar la escritura.
      lcd.print("SW=ON - ADC="); lcd.print(valorPA0);	// Escribe "SW 'OK'-ADC=" + el valor de "PA0"
      lcd.print("    ");								// Borra Celdas posteriores sobrantes
    }													// Salto a 'SVL02-B'
  } else {  // SVL02-B " tiempo de muestra de "SW=ON - ADC=" y al finalizar este muestra "presionar SWITCH     " "
    if (millis() - tiempoInicioLínea1 >= tiempoLínea1) {	// Si es 'False' salto a '"FIN" de SVL02'
      mostrandoPA0 = false;										// Fin del ciclo 'PA0'
      lcd.setCursor(0, 1);
      lcd.print("presionar SWITCH     ");						// Imprimir texto Inicial
    }  // SVL02-C
  }  // "FIN" de SVL02
}
//
```
