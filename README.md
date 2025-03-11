**v1.53** ## NEW CALIBRATION MENU ##

**Mejoras:**
- Reorganización de variables y funciones
- Entornos de desarrollo Simulación y Real
- Simulación básica de lectura y seteo de V e I
- Borrado de entrada por dígito en ResetInputValue()
- Mejora de Simulación diagram.jsom
- Rediseño del cambio de modo V, I y B con funcion ChangeMode(Modetoset)
- Reingenieria de Calibración por menu.
- Simplificacion de lecturas y escrituras de EEPROM con rescritura de valores "nan"
- Mode para a char en lugar de String.
- En Calib. Se indica Fail cuando algun factor llega a limite permitido.
- Simula carga variable.
- Mejora de editado e ingreso de valores.
- Presets del 1 al 9 sin enter, 0 para Menu de Calibración.

**Fixes:**
- En modo Preset o Cal, no borra valores de entrada, dando tiempo a corregir o cargar un valor.
- En Calib. quedaba el cursor en I cuando se calibro V.
- En Calib. se asegura que P1 y P2 se tomen para el modo seleccionado.

**Bugs:**
- No hay protección de OVP u OCP.

**Trabajando:**
- No mas para este release

**A Trabajar:**
- No mas para este release

**En Cola:**

**Posibles Mejoras SW:**
- Parar btn MEM a Shift, los preset sean shift + numero
- Unificar la estructura de datos de calibracion?
- Valores maximos seteables?
- Opcion de mostrar valores de calibracion?

**Posibles Mejoras HW:**
- Imprimir botones con extrusor de 0.2mm
- Ver de agregar un RELE para seleccionar bobina secundaria para V menor a 15V (tener en cuenta caidas de MOSFET por regulación)
- Control de los reles de output
- Cambiar/mejorar resistencia de sensado, usando las de DCload que sobraron. 
