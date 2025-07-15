https://github.com/ajdepaoli/MODULOS-ARDUINO/tree/main/DISPLAY/LCD_HD44780_2x16_Keypad_Shield_6SW/STM32

Este repositorio contiene módulos técnicos para Arduino, desarrollados y documentados bajo el enfoque ESTRUCTURADO.

TEST para los siguientes MODULOS:

MODULOS a TESTEAR:
#1 "STM32" con STM32F103C8T6 (128K)
#2 "I2C"
#3 "LCD Keypad Shield"

Otros MODULOS y componentes:
#4 Fuente alimentación 220VCA // 12VDC 2A
#5 Fuente para Protoboard: Entrada 7VDC a 12VDC; salidas 3.3V y 5V Máximo NO alimentar por el USB.
#6 Convertidor Doble Bidireccional de señal 3.3V <-> 5V (El de la foto, es de MADE in HOME)
#7 Interfaz "ST-LINK V2"
#8 Led x 1, en lo posible de alta eficiencia.
#9 Resistencia 1/4W 1K 1% o MFR para el led.
#10 Resistencia 1/4W 5.6K 1% o MFR para 'A0' a GND, esto reduce la salida del "LCD Keypad Shield" a 3.3V +- ya que está alimentado a 5VCC.
#11 Cables DUPONT varios MM y HH.
#12 Cables de uso telefónico, vienen de varios colores y son muy baratos.

Les debo el SCH de todo lo puesto en el PROTOBOARD, espero que la foto les sirva.

Tomen NOTA de los valores ADC ya que les va servir para diferentes SKETCH's.

ATENCION: al colocar la placa STM32 en el protoboard, traten de tapar los agujeros laterales, con puentes o cruzando cables, ya que si van a sacarla pueden sin querer errarle a una posición y quizas la quemen.
