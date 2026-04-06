# Fuente Lineal Programable

Firmware para una fuente lineal programable basada en Arduino Nano, con control independiente de tensión y corriente, presets, calibración a dos puntos, protecciones configurables y una interfaz local con LCD `20x4`, encoder y keypad.

## Qué hace

Este proyecto controla una fuente lineal de laboratorio con:

- modo de ajuste de tensión `CV`
- modo de ajuste de corriente `CI`
- presets rápidos de tensión y corriente
- menú de configuración en pantalla
- menú `Protection` con `OVP` y `OCP` configurables
- calibración de sensado y salida por dos puntos
- guardado de factores de calibración en EEPROM
- simulación en Wokwi y entorno para hardware real

## Interfaz

La interfaz principal muestra:

- mensajes y temperatura en la línea superior
- valores instantáneos de `V` e `I`
- potencia instantánea
- campo de ingreso numérico `Set V:` o `Set I:`
- campo `Real:` durante calibración
- campo `Set VP:` o `Set CP:` al editar protecciones

Con la tecla `S` se accede a funciones contextuales:

- `S + 1..6`: memorias de presets
- `S + 0`: menú `Config.`

## Menús actuales

`Config.` incluye:

- `Protection`
- `Calibration`
- `Exit`

`Protection` permite:

- `Set OVP`
- `Set OCP`
- `Exit`

`Calibration` permite:

- `Cal V`
- `Cal I`
- `Load Cal`
- `Save Cal`
- `Back`

## Protecciones

El firmware incorpora dos protecciones configurables:

- `OVP`: over-voltage protection
- `OCP`: over-current protection

Por defecto:

- `OVP = 30.0 V`
- `OCP = 5.0 A`

Comportamiento actual:

- si el valor instantáneo supera `OVP * 1.03`, la salida se deshabilita con mensaje `OFF: OVP!`
- si el valor instantáneo supera `OCP * 1.03`, la salida se deshabilita con mensaje `OFF: OCP!`
- el seteo de `V` e `I` también queda limitado por `OVP/OCP` para evitar que el propio set dispare la protección
- `OVP/OCP` no se guardan en EEPROM y vuelven a sus valores por defecto al reiniciar

## Calibración

La calibración usa dos puntos y distingue correctamente entre:

- sensado: `real = medido * factor + offset`
- salida: cálculo inverso del comando necesario para obtener el valor deseado

Durante calibración:

- el valor de abajo se interpreta como `Real:`
- el ajuste de salida se hace con encoder o teclas de dirección
- el valor real medido se ingresa por keypad

## Entornos

El proyecto tiene dos entornos en `platformio.ini`:

- `sim`: simulación
- `real`: hardware real

## Estado

Versión actual: `v1.54`

Pendientes principales:

- seguir optimizando RAM/Flash
- evaluar persistencia futura de otros parámetros configurables si realmente aporta valor
- revisar protecciones adicionales y límites avanzados
