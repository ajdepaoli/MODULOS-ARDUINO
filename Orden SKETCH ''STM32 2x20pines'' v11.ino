// ########### DATOS INICIALES ###########
// Nombre: aca va el nombre
// Programa para: nnnnnnnnnnn
// Módulo: STM32F103C8T6 (128K)
// Creador y programador: Alejandro Jorge Depaoli
// Ayudante: 
// Fecha de Inicializacion: Fecha
// Versión: nnnnnn
// Ultima Revisión: xx/06/2025
//
// ### PARAMETROS Básicos para "ARDUINO-IDE" -> STM32F103C8T6 Blue Pill <- por ST-LINK V2 ###
// NOTA en la PCB los pines de BOOT deben estar ambos es "0"
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
// ### PARAMETROS Básicos para "ARDUINO-IDE" -> STM32F103C8T6 Blue Pill <- por USB ###
// NOTA en la PCB los pines de BOOT deben estar ¿¿¿¿?????
// Placa: "Generic STM32F1 series"								>	SELECIONAR este ITEM
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
// IA00
// Ж Ж Ж Ж Ж --- BIBLIOTECAS --- Ж Ж Ж Ж Ж
//
// IA01
// Ж Ж Ж --- TIEMPO Y SISTEMA --- Ж Ж Ж
// Para delays precisos usar delay(), delayMicroseconds(), o HardwareTimer
#include <STM32FreeRTOS.h>           // ⏱️ Opcional: Multitarea (FreeRTOS) en placas STM32
#include <STM32LowPower.h>           // 🔋 Sleep / deepSleep / wakeUpTime para bajo consumo
//
// IA02
// Ж Ж Ж --- MEMORIA Y ALMACENAMIENTO --- Ж Ж Ж
#include <EEPROM.h>              // Acceso sencillo a EEPROM -- "Usable con core Arduino moderno (simulada en Flash)"
//
// IA03
// Ж Ж Ж --- FUNCIONES ESTÁNDAR (LIBC) --- Ж Ж Ж
#include <stdlib.h>              // malloc(), free(), rand(), atoi(), etc.
#include <string.h>              // strcpy(), strcmp(), memset(), etc.
#include <math.h>                // sin(), cos(), sqrt(), pow(), etc. -- (hardware FPU en algunos modelos)
//
// IA04
// Ж Ж Ж --- COMUNICACIONES --- Ж Ж Ж
#include <Wire.h>                // I²C: sensores, RTC, expanders
#include <SPI.h>                 // SPI: memorias, pantallas, etc. -- ver item "Ж Ж Ж PINES de: COMUNICACION Ж Ж Ж"
#include <SoftwareSerial.h>      // UART adicional por software -- OJO -- Funciona solo en algunas PCB's. Usar "HardwareSerial"
// HardwareSerial Serial1(PA10, PA9);   // Ejemplo de puerto UART1 en STM32 (RX, TX)
//
// IA05
// Ж Ж Ж --- PERIFÉRICOS Y DISPLAYS --- ###
#include <LiquidCrystal.h>       // LCD estándar (HD44780) -- Compatible para LCD paralelo requiere 6 pines GPIO
#include <LiquidCrystal_I2C.h>   // ✅ LCD I²C -- Autor: Frank de Brabander — para módulos con PCF8574
#include <Servo.h>               // Control de servomotores (PWM)  -- Compatible (usando Timer internos STM32)
#include <Adafruit_GFX.h>        // 🎨 Base para displays gráficos (opcional)
#include <Adafruit_SSD1306.h>    // 🖥️ OLED 128x64 (opcional)
//
//
//
// "IB00"
// Ж Ж Ж Ж Ж Ж Ж --- DEFINICIÓN de los PINES (ENTRADAS y SALIDAS) --- Ж Ж Ж Ж Ж Ж Ж
//
// "IB01"
// Ж Ж Ж SALIDAS DIGITALES Low/High o 0V/3.3V Ж Ж Ж
// NOTA: En STM32 los niveles lógicos son 0V = Low y 3.3V = High (NO toleran 5V en general)
// Por lo general se usa "digitalRead(PXx)" con el nombre definido en el pin mapping
// EJ.: Define el pin con el nombre "NV_inD0", donde: 
//      "NV" = Nombre de la Variable
//      "_in" = subtipo funcional de la variable
//      "D0" = entrada Digital Nº 0
//		"PNx": "PAx" o "PBx"  = donde "Nx" es el nombre del pin en Módulo STM32
// ⚠️ IMPORTANTE: Las entradas digitales NO toleran 5 V directamente
//		Usar divisor resistivo y zener limitante si se conecta a salidas TTL de 5VCC.
// Definición      		// Descripción o periférico asociado
#define NV_inD0 PA0		// Entrada digital 0	—	ADC_IN0, TIM2_CH1, WKUP
#define NV_inD1 PA1		// Entrada digital 1	—	ADC_IN1, TIM2_CH2
#define NV_inD2 PA2		// Entrada digital 2	—	ADC_IN2, USART2_TX, TIM2_CH3
#define NV_inD3 PA3		// Entrada digital 3	—	ADC_IN3, USART2_RX, TIM2_CH4
#define NV_inD4 PA4		// Entrada digital 4	—	ADC_IN4, SPI1_NSS
#define NV_inD5 PA5		// Entrada digital 5	—	ADC_IN5, SPI1_SCK, TIM2_CH1
#define NV_inD6 PA6		// Entrada digital 6	—	ADC_IN6, SPI1_MISO, TIM3_CH1
#define NV_inD7 PA7		// Entrada digital 7	—	ADC_IN7, SPI1_MOSI, TIM3_CH2
#define NV_inD8 PB0		// Entrada digital 8	—	ADC_IN8, TIM3_CH3
#define NV_inD9 PB1		// Entrada digital 9	—	ADC_IN9, TIM3_CH4
// Se pueden definir otros Pines como Entrada, estos son los que están por defecto. Verificarlo con el DATASHEET.
//
//
// "IB02"
// Ж Ж Ж ENTRADAS ANALOGICAS Low/High o 0V/3.3V Ж Ж Ж
// Por lo general se usa "digitalWrite(PXx, HIGH/LOW)" para la escritura del pin
// EJ.: Define el pin PNxx con el nombre "NV_outPNxx", donde: 
//      "NV" = Nombre de la Variable
//      "_out" = representa el subtipo funcional de la variable
//      "PNxx": "PAx" o "PBxx" = donde "Nxx" es el nombre del pin en Módulo STM32
// Definición        		// Descripción funcional o periférico asociado
#define NV_out0  PA0 		// Salida digital 0
#define NV_out1  PA1 		// Salida digital 1
#define NV_out2  PA2 		// Salida digital 2 (UART2_TX si no se usa Serial2)
#define NV_out3  PA3 		// Salida digital 3
#define NV_out4  PA4 		// Salida digital 4 (SS opcional para SPI1, puede ser otro pin)
#define NV_out5  PA5 		// Salida digital 5 (SCK si se usa SPI1)
#define NV_out6  PA6 		// Salida digital 6 (MISO opcional)
#define NV_out7  PA7 		// Salida digital 7 (MOSI si se usa SPI1)
#define NV_out8  PB0 		// Salida digital 8
#define NV_out9  PB1 		// Salida digital 9
#define NV_out10 PB10		// Salida digital 10 (UART3_TX si no se usa Serial3)
#define NV_out11 PB11		// Salida digital 11
#define NV_out12 PB12		// Salida digital 12 (SS opcional para SPI2, puede ser otro pin)
#define NV_out13 PB13		// Salida digital 13 (SCK SPI2)
#define NV_out14 PB14		// Salida digital 14 (MISO SPI2)
#define NV_out15 PB15		// Salida digital 15 (MOSI SPI2)
//
//
// "IB03"
// Ж Ж Ж ENTRADAS ANALOGICAS '0V a 3.3 V' Ж Ж Ж
// ⚠️ IMPORTANTE: Entradas analógicas NO toleran 5 V — aplicar divisor resistivo si la señal supera 3.3 V
// Por lo general se usa "analogRead(PNx)" para la lectura del pin
// EJ.: Define el pin Ax con el nombre "NV_inAx", donde: 
//      "NV" = Nombre de la Variable
//      "_in" = representa el subtipo funcional de la variable
//      "PAx" = donde "Ax" es el nombre del pin en Módulo STM32
// Definición			// Canal ADC asociado
#define NV_inA0  PA0	// ADC_IN0
#define NV_inA1  PA1	// ADC_IN1
#define NV_inA2  PA2	// ADC_IN2
#define NV_inA3  PA3	// ADC_IN3
#define NV_inA4  PA4	// ADC_IN4
#define NV_inA5  PA5	// ADC_IN5
#define NV_inA6  PA6	// ADC_IN6
#define NV_inA7  PA7	// ADC_IN7
#define NV_inA8  PB0	// ADC_IN8
#define NV_inA9  PB1	// ADC_IN9
//
//
// "IB04"
// Ж Ж Ж COMPARACIÓN ANALÓGICA (por software) Ж Ж Ж
// STM32F103C8T6 NO posee un comparador analógico dedicado como el ATMega328P
// Sin embargo, se puede emular la comparación entre dos señales analógicas por software:
//        Si ADC_0 > ADC_1 → resultado lógico HIGH
//        Si ADC_0 < ADC_1 → resultado lógico LOW
// EJ.: Define los pines con nombres simbólicos como:
#define NV_cmpAIN0 PA0   // Entrada analógica positiva (ADC_IN0)
#define NV_cmpAIN1 PA1   // Entrada analógica negativa (ADC_IN1)
// ⚙️ Función ejemplo para comparación directa
bool compararAnalogicamente() {
    int ain0 = analogRead(NV_cmpAIN0);
    int ain1 = analogRead(NV_cmpAIN1);
    return (ain0 > ain1);  // true si "positivo" > "negativo"
}
// NOTAS:
// 1- Requiere inicialización de ADC (por defecto en STM32 core)
// 2- Precisión depende de tiempo de muestreo y resolución (12 bits = 0 a 4095)
// 3- Usar oversampling o promedio si hay ruido analógico
// 4- Alternativa: configurar interrupción si alguno supera umbral crítico
// 5- También existe sensor de temperatura interno si se necesita monitoreo térmico
//
//
// "IB05"
// Ж Ж Ж SALIDAS ANALOGICAS Ж Ж Ж
// STM32F103C8T6: NO tiene DAC.
// Se puede emular con la función "PWM", agregando filtros RC.
// Definir el nombre de la función como "_outdac" en caso de simular una Salida Analogica.
// #define PWMx_outdac  PAx		// TIM2_CHx — PWMx
//
//
// "IB06"
// Ж Ж Ж SALIDAS PWM Ж Ж Ж
// Pines PWM disponibles dependen del Timer asociado y canal
// Frecuencia PWM predeterminada varía según core. Se puede configurar por Timer si se requiere precisión
// Usar analogWrite(PXx, valor); donde valor ∈ [0, 255]
// EJ.: Define el pin PNx con el nombre "PWMx_out PNx", donde: 
//      "PWM" = Nombre de la Variable
//      "_out" = representa el subtipo funcional de la variable
//      "PNx": "PAx" o "PBx" = donde "Nx" es el nombre del pin en Módulo STM32
// Definición simbólica		// Descripción funcional y origen PWM
#define PWM1_out  PA0		// TIM2_CH1 — PWM1 (ADC conflict si se mide A0)
#define PWM2_out  PA1		// TIM2_CH2 — PWM2
#define PWM3_out  PA2		// TIM2_CH3 — PWM3
#define PWM4_out  PA3		// TIM2_CH4 — PWM4
#define PWM5_out  PA6		// TIM3_CH1 — PWM5 (compartido con SPI1_MISO)
#define PWM6_out  PA7		// TIM3_CH2 — PWM6 (compartido con SPI1_MOSI)
#define PWM7_out  PB0		// TIM3_CH3 — PWM7
#define PWM8_out  PB1		// TIM3_CH4 — PWM8
// ⚠️ NOTAS:
// 1- Algunos de estos pines también son entradas analógicas o pines SPI → verificar conflictos
// 2- Para mejor resolución o frecuencia, se puede usar control directo con HardwareTimer
// 3- PWM por software (analogWrite) solo en placas con soporte desde el core Arduino STM32
// 4- Se recomienda mantener la carga posterior al filtro RC por debajo de 10kΩ para evitar deformación
// 5- Relación de Pines PWM con Timers — para uso con 'HardwareTimer'
//			TIM2_CH1 → PA0     TIM2_CH2 → PA1     TIM2_CH3 → PA2     TIM2_CH4 → PA3
//			TIM3_CH1 → PA6     TIM3_CH2 → PA7     TIM3_CH3 → PB0     TIM3_CH4 → PB1
//
//
// "IB07"
// Ж Ж Ж COMUNICACION Ж Ж Ж
// NO se DECLARAN aquí, solo se toma NOTA para tenerlos en CUENTA.
// Se activan al incluirla en la BIBLIOTECA --> #nnn <NNNN.h> y llamar a NNNN.begin();
//
// Ж Ж SPI (Serial Peripheral Interface) Ж Ж
// Biblioteca --> #include <SPI.h> --> SPI.begin()
// Por defecto:
// "SCK"   --> pin: PA5		// Reloj del bus
// "MISO"  --> pin: PA6		// Entrada de datos
// "MOSI"  --> pin: PA7		// Salida de datos
// "SS"    --> pin: cualquier GPIO definido por el usuario (ej.: PA4)
//
// Opcional: usar SPI2 o redefinir SPI personalizado:
// SPIClass mySPI(SPI2, /*MISO*/ PB14, /*MOSI*/ PB15, /*SCK*/ PB13);
// mySPI.begin();
//
// Ж Ж I²C (Wire) Ж Ж
// Biblioteca --> #include <Wire.h> --> Wire.begin()
// Pines por defecto del bus I²C1 en Blue Pill:
// "SDA"   --> PB7
// "SCL"   --> PB6
// Nota: pueden variar según modelo; verificar en datasheet o CubeMX
//
// Ж Ж UART / USART (Seriales Hardware) Ж Ж
// Biblioteca base ya incluida por defecto en (Arduino.h) --> Serial.begin(baudrate);
// Serial:        si está habilitado como CDC USB → comunicación por USB virtual
// Serial1 (HW):  TX = PA9 , RX = PA10
// Serial2 (HW):  TX = PA2 , RX = PA3
// Serial3 (HW):  TX = PB10, RX = PB11
//
// ¡OJO! SoftwareSerial solo funciona en algunas placas. Mejor usar 'HardwareSerial':
// Ejemplo:
// HardwareSerial SerialX(PA10, PA9);  // RX, TX
// SerialX.begin(9600);
//
// Ж Ж CAN Bus Ж Ж
// SOLO se puede programar atravez de ST-LINK
// Requiere librería específica, y transceptor externo (ej. MCP2551)
// CAN_RX --> PA11
// CAN_TX --> PA12
//
// Ж Ж USB Ж Ж
// Si se configura como CDC (USB virtual COM), entonces:
// "D+" --> PA12
// "D−" --> PA11
// Solo disponible en Arduino-IDE si dispone de "USB support: "CDC" o "CDC no Generic Serial" "
//
//
// "IB08"
// Ж Ж Ж INTERRUPCIONES EXTERNAS 'EXTI' Ж Ж Ж
// NO se DECLARAN aquí, solo se toma NOTA para tenerlos en CUENTA.
// Las interrupciones 'EXTI' deben configurarse en 'setup()' o mediante 'attachInterrupt()'
// EJ.: Define el pin PNx con el nombre "NV_extix PNx", donde: 
//      "NV" = Nombre de la Variable
//      "_extix" = representa el subtipo funcional de la variable con su correspondiente Nº.
//      "PNx": "PAx" o "PBx" = donde "Nx" es el nombre del pin en Módulo STM32
// Pines compatibles con EXTI 0 a 15 (mapeables a PAx, PBx o PCx)
#define NV_exti0  PA0  // EXTI0  — Puede usarse como 'wakeup'
#define NV_exti1  PA1  // EXTI1
#define NV_exti2  PA2  // EXTI2
#define NV_exti3  PA3  // EXTI3
#define NV_exti4  PA4  // EXTI4
#define NV_exti5  PA5  // EXTI5
#define NV_exti6  PA6  // EXTI6
#define NV_exti7  PA7  // EXTI7
#define NV_exti8  PB0  // EXTI8
#define NV_exti9  PB1  // EXTI9
// ⚠️ NOTA: Cada línea EXTI (0–15) solo puede estar asignada a UN pin al mismo tiempo (PAx, PBx o PCx)
// 💡 attachInterrupt(digitalPinToInterrupt(PAx), ISR_function, mode);  // mode: RISING, FALLING, CHANGE
//
//
// "IB09"
// Ж Ж Ж TIMERs y PWM controlados por hardware Ж Ж Ж
// NO se DECLARAN aquí, solo se toma NOTA para tenerlos en CUENTA como salidas PWM o controladas por hardware.
// Los canales TIMx_CHy no necesitan ser definidos aquí — se configuran por código o HardwareTimer
// 📍 Ubicación de declaración e inicialización:
// Las instancias de 'HardwareTimer' deben inicializarse en el bloque 'setup()' o en secciones dedicadas,
// 		EJEMPLO 1 de USO en 'setup()':
//  			HardwareTimer timer2(TIM2);
//   			timer2.setPWM(PA0, 1000, 0.5); // Frecuencia: 1 kHz, Duty: 50%
// 		EJEMPLO 2 de USO en 'setup()':
//  			HardwareTimer timer2(TIM2);
//				timer2.setPWM(PA0, frecuencia_Hz, duty_0a1); // duty: valor entre 0.0 y 1.0 (ej.: 0.75 → 75%)
// Este bloque lista los canales disponibles por Timer (uso avanzado)
// EJ.: Define el pin PNx con el nombre "TIMx_CHx PNx", donde: 
//      "TIMx" = Nombre de la Variable con su correspondiente Nº de TIMer
//      "_CHy" = 'CHanel' con su correspondiente Nº de Canal.
//      "PNx": "PAx" o "PBx" = donde "Nx" es el nombre del pin en Módulo STM32
// TIM1 — 4 canales (PWM avanzado: salidas complementarias, dead-time, etc.)
//		✔️ Especial para control de motores, drivers, o donde se requiera precisión de fase y sincronización
//		⚠️ Requiere cuidado en configuración avanzada si se activan salidas complementarias (CH1N, CH2N...)
//		✔️ CH1N a CH3N (salidas invertidas) están disponibles en pines alternativos (consultar RM0008 / pinout)
//  #define TIM1_CH1  PA8   // Salida PWM CH1
//  #define TIM1_CH2  PA9   // Salida PWM CH2
//  #define TIM1_CH3  PA10  // Salida PWM CH3
//  #define TIM1_CH4  PA11  // Salida PWM CH4
// TIM2 — 4 canales (PWM o captura/compare)
//  #define TIM2_CH1  PA0   // También ADC_IN0
//  #define TIM2_CH2  PA1
//  #define TIM2_CH3  PA2
//  #define TIM2_CH4  PA3
// TIM3 — 4 canales (ideal para PWM)
//  #define TIM3_CH1  PA6   // También SPI1_MISO
//  #define TIM3_CH2  PA7   // También SPI1_MOSI
//  #define TIM3_CH3  PB0
//  #define TIM3_CH4  PB1
// TIM4
// ⚠️ STM32F103C8T6 también puede incluir TIM4 (consultar datasheet y variante de placa utilizada)
// NOTAS:
// ⚠️ - Estos pines pueden compartir funciones SPI, ADC o EXTI
// ⚠️ - Planificar uso para evitar conflictos
//
//
// "IB10"
// Ж Ж Ж PROGRAMACIÓN Y BOOTLOADER Ж Ж Ж
// Este microcontrolador no utiliza "fuses" como los AVR ni los terminos ISP o ICSP.
// La configuración de arranque se define mediante:
//  - Celdas de opción (Option Bytes) grabadas desde ST-Link o software dedicado.
// BOOT0 (pin BOOT0 del Módulo) → Selecciona modo de arranque (ver tabla de arranque):
//		conectado a BOOT0/PA15 del micro con R3=100KΩ en algunos casos.
//		• BOOT0 = 0 (LOW 0V) → Arranque desde memoria Flash interna (modo normal)
//		• BOOT0 = 1 (HIGH 3.3V) → Arranque desde sistema de fábrica (System Memory: USART1, DFU USB si está cargado)
// BOOT1 (pin BOOT1 del Módulo) →  Selecciona modo de arranque (ver tabla de arranque):
//		conectado a BOOT1/PB2 del micro con R4=100KΩ en algunos casos.
//
// 🧩 Tabla de arranque (STM32F103)
//	| BOOT1 | BOOT0 | Modo de arranque                  | 
//	|   0   |   0   | Flash interna (modo normal)       | 
//	|   0   |   1   | System Memory (USART1, DFU, etc.) | 
//	|   1   |   0   | SRAM interna (debug)              | 
//	|   1   |   1   | SRAM interna (debug)              | 
//
// RESET  (pin NRST en micro) (pin 'R' en Modulo) → Pin de reinicio del sistema (pull-up interno activo)
//
// NOTA:
// • Para programar con ST-Link: usar SWDIO y SWCLK (pines PA13 y PA14), BOOT0 = 0 o 1 NO importa
// • Para usar modo DFU por USB, se requiere cargar un bootloader compatible (ej.: Maple).
//   	Para activarlo: establecer BOOT0 = 1 antes del reset (BOOT desde System Memory).
// • Para arranque confiable, Conectar BOOT0 a GND mediante resistencia de 10KΩ (pull-down externo recomendado)
// ⚠️ En Blue Pill, BOOT1 = 0 fijo → Solo se manipula BOOT0 para cambiar el modo de arranque.
// ⚠️ En muchos módulos Blue Pill, R3 (BOOT0) está a GND, y R4 (BOOT1) también a GND.
//		 Por eso ambos tienden a 0 al encender el sistema.
//
//
// "IB11"
// Ж Ж Ж ALIMENTACIÓN  Ж Ж Ж
// Nombre	Nº del pin del Modul	--> Función									Tipo
// "G"   	pin  1					--> 0V o Tierra								Entrada o Salida
// "G"   	pin  2					--> 0V o Tierra								Entrada o Salida
// "Vbat"	pin 20					--> 3.3V de bateria de Back-Up o Stand-By	Entrada
// "3.3" 	pin 21					--> 3.3V									Entrada o Salida
// "G"   	pin 22					--> 0V o Tierra								Entrada o Salida
// "5V"  	pin 23					--> 5V desde USB							Salida
//
//
// "IB12"
// Ж Ж Ж REFERENCIAS ADC INTERNA Ж Ж Ж
// STM32F103 incluye referencias internas accesibles por software:
//   - VREFINT (~1.20 V) → canal ADC_IN17 (no expuesto físicamente)
//   - Sensor de temperatura → canal ADC_IN16
// Recurrir a bibliografía sobre "VREFINT" (Referencia Interna de Voltaje)
// ⚠️ Requieren configuración adicional del ADC para ser leídos.
// 📚 Ver documentación oficial de STM32 o el Reference Manual (RM0008) sección ADC
//
//
//
//
// "IC00"
// Ж Ж Ж Ж Ж ------- VARIABLES ------- Ж Ж Ж Ж Ж
//
//
// "IC01"
// Ж Ж Ж ----- OBJETO GLOBALES ----- Ж Ж Ж
// Objetos de clase usados en todo el programa (LCD, RTC, sensores, etc.)
// Siempre primero.
// Son los módulos físicos del sistema embebido (LCD, sensores, EEPROM, etc.) y suelen tener constructores que se ejecutan al inicio.
// (lo siguiente son ejemplos, quizas falten y aclararlos mejor)
// Los siguientes son idelaes cuando solo lo usamos 1 vez o en en una repetitiva", imprimir en un display.
// String str01 = " CONTROL de DIR  ";  // Colocar Espacios adelante y atrás. MAXIMO 40 Caracteres"
// String str02 = " del BUS del I2C ";  // Ambos String deben tener la misma longitud.
// String str03 = "CAMBIAR Dir:";  // Para usar varias veces
// String msgDireccionIncorrecta = "CAMBIAR Dir:";  // Para usar varias veces (Igual al anterior, pero con nombre es mas fácil)
//
//
// "IC02"
// Ж Ж Ж ----- ESCALARES GLOBALES ----- Ж Ж Ж
// Variables primitivas (int, bool, unsigned long, etc.) con alcance global
// Usadas por varias subrutinas o para mantener estados (como ledState, millis() de control, flags de error, etc.).
bool NombreDeLaVariable = valor;                       // valor = false/true -- bits = 1 -- ¿%d? -- 
const int NombreDeLaVariable = valor;                  // valor = -32,768 a 32,767 -- ¿Significado cortisimo para recordarlo?
int NombreDeLaVariable_val = valor;                    // valor = -32,768 a 32,767 -- ¿Significado cortisimo para recordarlo?
unsigned long NombreDeLaVariable_val = valor;          // valor = 0 a 4,294,967,295 -- ¿Significado cortisimo para recordarlo?
// con modificador "static"
static bool NombreDeLaVariable = valor;                // valor = false/true -- ¿Significado cortisimo para recordarlo?
static unsigned long NombreDeLaVariable_val = valor;   // valor = 0 a 4,294,967,295 -- ¿Significado cortisimo para recordarlo?
//
//
// "IC03"
// Ж Ж Ж ----- LISTAS ----- Ж Ж Ж (puede cambiar)
// Arrays, buffers, cadenas fijas, listas de datos (char[], int[], etc.)
// Muy útil si manejás estructuras como:
// Ж LISTA "NonbreDeLaLista01" (aclarar funcion)  Ж
enum nombre {
    NonbreDeLaLista01_Abreviación_01_DelNonbreAlQueCorrespondeElValor = Valor0,      // valor = 0 a 4,294,967,295
    NonbreDeLaLista01_Abreviación_02_DelNonbreAlQueCorrespondeElValor = Valor0,      // valor = 0 a 4,294,967,295
    NonbreDeLaLista01_Abreviación_xx_DelNonbreAlQueCorrespondeElValor = Valor0,      // valor = 0 a 4,294,967,295
//EJEMPLO:
//  ERR_pst = 3,      // PRESOSTATO mal funcionamiento
//  ERR_ven = 4,      // "VENTILACION" Activada
//  ERR_cal = 5,      // "CALEFACCION" Activada
//  ERR_nn14 = 14 o Mayores ¡¡NO se deben USAR!!
};
//
//
// "IC04"
// Ж Ж Ж ----- Variables Flags y estados globales ----- Ж Ж Ж
// Variables para manejo de errores, estados, alarmas, etc.
//
//
// "IC05"
// Ж Ж Ж ----- Variables LOCALES ----- Ж Ж Ж
// Declaradas dentro de funciones/subrutinas (en `setup()`, `loop()` o funciones propias)
// No se declaran globalmente, este se reserva para documentar dónde y cómo usarlas (por ejemplo, si usás muchas en loop() o setup()).
//
//
//
//
// "ID00"
// %& %& %& %& %& %& %& ------ CONSTANTES Y MACROS ------ %& %& %& %& %& %& %&
//    ID01 → Constantes literales (const, #define)
//    ID02 → Enumeraciones (enum)
//    ID03 → Macros complejas
//
//
//
//
// "IE00"
// ϸ ϸ ϸ ϸ ϸ ϸ ϸ ------- S U B R U T I N A S ------- ϸ ϸ ϸ ϸ ϸ ϸ ϸ ϸ
//
// "IE01"
// ϸ ϸ ϸ ϸ ϸ ----- S.R.: USO de " EEPROM " ----- ϸ ϸ ϸ ϸ ϸ
//
//
// "IE02"
// ϸ ϸ ϸ ϸ ϸ ----- S.R. de USO  UNICO ----- ϸ ϸ ϸ ϸ ϸ
// Son aquellas que se corren una sola vez; lo general en "setup()" o son llamadas por única vez en loop().
//
// ϸ ϸ ϸ SubRutina "NN01" ϸ ϸ ϸ
// versión y otros datos: 
// Aclarar funcionamiento: 
void SubrutinaUU_NN01() {
  // Aquí va el código de ejecución.
} // "FIN" SubrutinaUU_NN01
//
//
// "IE03"
// ϸ ϸ ϸ ϸ ϸ ----- S.R.  en 2º PLANO PERMANENTES ----- ϸ ϸ ϸ ϸ ϸ
//
// ϸ ϸ ϸ --- SubRutina "NN01" --- ϸ ϸ ϸ
// versión y otros datos
// Aclarar funcionamiento.
void Subrutina2PP_NN01() {
  // Aquí va el código de ejecución.
} // "FIN" Subrutina2PP_NN01
//
//
// "IE04"
// ϸ ϸ ϸ ϸ ϸ ----- S.R.  en 2º PLANO NO PERMANENTES ----- ϸ ϸ ϸ ϸ ϸ
//
// ϸ ϸ ϸ --- SubRutina "NN02" --- ϸ ϸ ϸ
// versión y otros datos
// Aclarar funcionamiento.
void Subrutina2PNP_NN01() {
// Aquí va el código de ejecución.
} // "FIN" ϸͰϸͰ Subrutina2PNP_NN01 ϸ˧ϸ˧ 
//
//
// "IE05"
// ϸ ϸ ϸ ϸ ϸ ----- SUBRUTINAS de USO COMUN  ----- ϸ ϸ ϸ ϸ ϸ
//
// ϸ ϸ ϸ --- SubRutina "NN03" --- ϸ ϸ ϸ
// versión y otros datos
// Aclarar funcionamiento.
void SubrutinaUC_NN03() {
 // Aquí va el código de ejecución.
} // "FIN" de SubrutinaUC_NN03, RETORNO a donde fue llamada esta SubRutina.
//
//
//
//
// "IF00"
// Д Д Д Д Д Д Д ------- setup() ------- Д Д Д Д Д Д 
//
void setup() {
//
// Д Д Д Д Д ----- CONFIGURACION INICIAL ----- Д Д Д Д Д
//
// Д Д Д Configuración de los Pines como ENTRADAS Д Д Д
pinMode(NV_in1, INPUT);
pinMode(NV_inx, INPUT);
//
// Д Д Д Configuración de los Pines como SALIDAS Д Д Д
pinMode(NV_out1, OUTPUT);
pinMode(NV_outx, OUTPUT);
//
// Д Д Д Asegurar el Estado inicial "SEGURO" de las SALIDAS Д Д Д
digitalWrite(NV_out1, LOW);      // Pone la salida en LOW (Ejemplo)
digitalWrite(NV_out2, HIGH);     // Pone la salida en HIGH (Ejemplo)
digitalWrite(NV_outx, LOW/HIGH); // Pone la salida en LOW/HIGH (elegir el estado más conveniente)
//
// Д Д Д Comunicación con el MONITOR ¡ ¡ ¡ O J O ! ! ! con los pines Rx y Tx Д Д Д
Serial.begin(Baud);                         // Elegir la velocidad de Rx/Tx
Serial.println(">> INICIO DEL SISTEMA <<"); // Mostrará en la pantalla del monitor o PC la leyenda que hay dentro de (".....")
Serial.println("Inicializando pines...");   // Mostrará en la pantalla del monitor o PC la leyenda que hay dentro de (".....")
//
// Д Д Д Д Д ----- OTRAS Configuraciones ----- Д Д Д Д Д
// "" Activa el watchdog con 2s de límite ""
// wdt_enable(WDTO_2S);
//
// Д Д Д --- SUBRUTINAS setup()--- Д Д Д
// Aquí se pueden agregar subrutinas de cualquier tipo.
//
}
//
//
//
//
// "IG00"
// Ē Ē Ē Ē Ē Ē Ē ------- loop() ------- Ǝ Ǝ Ǝ Ǝ Ǝ Ǝ Ǝ 
// ¡ ¡ ¡ Aquí se desarrolla el programa o sistema de control PRINCIPAL ! ! !
//
void loop() {
//
// Comandos pre-subrutinas
// Aquí van las subrutinas de cualquier tipo.
// Comandos post-subrutinas
//
//
} // "FIN" --- RETORNA a loop() ---
// NOTA Este espacio siempre debe estar al final ver "NOTA FINAL" en.,.....
//
``` // Estas comillas son usadas como final del testo copiado para ayuda con la IA
// "IH00"
//


