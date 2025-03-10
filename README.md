**v1.53**

**Mejoras:**
- Reorganización de variables y funciones
- Entornos de desarrollo Simulación y Real
- Simulación básica dde lectura y seteo de V e I
- Borrado de entrada por dígito en ResetInputValue()
- Mejora de Simulación diagram.jsom
- Rediseño del cambio de modo V, I y B con funcion ChangeMode(Modetoset)
- En modo Preset o Cal, no borra valores de entrada, dando tiempo a cooregir o cargar un valor.

**Fixes:**
- En Calibración quedaba el cursor en I cuando se calibro V.

**Bugs:**
- Despues de P1, puedo setear un P2 de distinto modo
- Current Sns podría tener un margen del 1% para que permita llegar a los 5A

**Trabajando:**
- Reordenamiento del código.
- La calibracion es muy compleja, simplificar.

**A Trabajar:**
- Revisar el editado de ingreso de valores.

**En Cola:**

**Posibles Mejoras SW:**

**Posibles Mejoras HW:**
- Ver de agregar un RELE para seleccionar bobina secundaria para V menor a 15V (tener en cuenta caidas de MOSFET por regulación)
