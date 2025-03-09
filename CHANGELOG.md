// 1.00a Primera versión, Solo funciones de Modo V y settings con encoder
// 1.01a Incorporación del Keypad, funciòn basica 
// 1.02a Función Limits_check
// 1.10a Incorporacion de entrada de teclado y revisiòn de todas las funciones.
// 1.11a Cambios para notificaciones unificadas de mensajes, Si se presiona Enter in input setea 0.
// 1.12a Rutinas para ADC y DAC, primera prueba en Protoboard, correcciones varias de limites y funcionamiento
// 1.20b Primer Beta, No se ve el cursor, enter setea valor cero porque numbers se resetea a cero en la funcion de display settings
// 1.30b Me doy cuenta que el ADC1115 tiene que sensar si o si una tensiòn por arriba de GND, agrego inversor con TCL062CP, primera versión funcional.
// 1.31b Cambios en la gestión del cursor... display periodico en lugar de continuo. Emprolijo el código
// 1.32b Cbio sns de A a amp dif. para sensar el R Shunt en cualquier parte (posible agregado de otro MOSFET con R de protección.) Calibracion
// 1.33b Refresh de mensajes corregido, agreso sensor de temperatura y rutina
// 1.34b Cbio de Pinout para dar lugar al control de Reles para los Canales de salida, agregar rutinas de control de Fans y pulsadores para activar las salidas
// 1.35b Version para PCB, invertidos las entradas del ADC de V e I, descarto control de Reles de Salidas
// 1.36 Funcional con variables para zero offset de corriente y voltage
// 1.37 Protección de exceso de disipación de potencia en 2N3055 y chequeo de dacs y adc, verificacion de Calibración, flag de salud gral.
// 1.38 Muestra Temperatura del disipador si no hubo otros mensajes.
// 1.39 Memorias DC estandar
// 1.40 Calibracion solo de sensado, sin almac. en EEPROM. Ajustes en pos. de cursor
// 1.41 Guarda en la EEPPROM el factor de Calibración de sensados
// 1.42 Calibracion de out V e I con guardado en EEPROM
// 1.42b organización del código y funciones de Display
// 1.43 Cambio en el diplay de valores en el LCD
// 1.43b Cambio Variables int16_t adc0, adc1, adc2, adc3 por adci, adcv porque las estaba usando mal. Pruebo ADC modo diferencial. Pruebas con Offset
// 1.44 Flechas indicando que se esta seteando V o I, ajuste de mensajes de Preset y metodo de Calibracion, lo estaba haciendo mal.
// 1.45b Ajuste de mem preset y mensaje. Rename de bool mem y cal a mem_st y cal_st. Cambio funcion CheckCalibration a Save_Calibration y pongo SetIV en falso luego
// 1.45c Saco Op amp inversores (no funciona) y tomo valores asolutos de tensión y corriente. Pongo offset en cero y dejo de controlar en display si es menor que cero
// 1.46a Cambio método de calibracion a dos puntos.
// 1.46b Corrección de Bugs de Calibración. No controlo si se sensan valores negativos. Falta guardar en EEPRON el valor de Offset calibración de offset
// 1.46 Modifico rutinas calibración de offset, carga y grabado en EEPROM. Corrijo rutina de calibración para que recupere cal de EEPROM del modo que no fue calibrado
// 1.47b Mejora de mensajes de Debuguing por Serial Port. Aumento index a 7 si esta en modo Calibración para mas presción en la calibración.
// 1.48 Cambio el 2N3055 por dos TIP3055 por lo que cambio la protección de potencia y curva de Temperatura
// 1.49 Cbio. Mens, de Temp, saco dcmls en V_I_W_Display, Reorg. func. Temp_Check y Limit Check creo ResetInputValue y Manage_Display =  Display + Display_Timers. Comentado e indent.
// 1.50 Agrego variables de calibracion offset out, saco #define para constantes, SE CUELGA, demasiada memoria dinámica?
// 1.50a saco offset para seteo de corriente maxima de salida para reducir memoria dinamica, SE CUELGA pero lleva a mostrar DacV OK en el LCD
// 1.50b Acorto texto de Serial.Print() ya que se almacenan en SRAM, con F() se graban en la FLASH. FUNCIONA, ahorro mucha memoria
// 1.50.1b Cambios en la controladora, cambian factores de corriente de diseño para SNS y OUT
// 1.51 Compilado usando VS Code, la condicion de Preset no permite ver el cursor con los cambios del encoder
// 1.52 Mejora en el refresh de valores seteados en display. Resta ver porque el boton de clear no resetea el cursor, ver si es porque no se llama a la fucnion de display
// 1.53 Agrego #define WOKWI_SIMULATION para que no compile las librerias de Adafruit en la simulación de Wokwi y no tire error. Saco el fastincrement del encoder