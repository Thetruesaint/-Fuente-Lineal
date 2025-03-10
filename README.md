**v1.53**

**Mejoras:**
- Reorganización de variables y funciones
- Entornos de desarrollo Simulación y Real
- Simulación básica dde lectura y seteo de V e I
- Borrado de entrada por dígito en ResetInputValue()
- Mejora de Simulación diagram.jsom
- Rediseño del cambio de modo V, I y B con funcion ChangeMode(Modetoset)
- En modo Preset o Cal, no borra valores de entrada, dando tiempo a corregir o cargar un valor.
- Reingenieria de Calibración por menu.
- Simplificacion de lecturas y escrituras de EEPROM con rescritura de valores "nan"
- Mode para a char en lugar de String.
- En Calib. Se indica Fail cuando algun factor llega a limite permitido.

**Fixes:**
- En Calib. quedaba el cursor en I cuando se calibro V.
- En Calib. se asegura que P1 y P2 se tomen para el modo seleccionado.

**Bugs:**
- Current Sns podría tener un margen del 1% para que permita llegar a los 5A
- No hay warning si current o voltage es mayor al seteado.
- Queda el mensaje de warning sale al salir de Preset

**Trabajando:**
- Test de nuevo modo de Calibracón.

**A Trabajar:**

**En Cola:**

**Posibles Mejoras SW:**
- Revisar el editado de ingreso de valores.
- Parar btn MEM a Shift, los preset sean shift + numero
- Unificar la estructura de datos de calibracion?
- Valores maximos seteables?
- Opcion de mostrar valores de calibracion?

**Posibles Mejoras HW:**
- Imprimir botones con extrusor de 0.2mm
- Ver de agregar un RELE para seleccionar bobina secundaria para V menor a 15V (tener en cuenta caidas de MOSFET por regulación)
- Control de los reles de output
- Cambiar/mejorar resistencia de sensado, usando las de DCload que sobraron. 
