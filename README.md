# Fuente Lineal Programable

Firmware para una fuente lineal programable basada en Arduino Nano, con control independiente de tensión y corriente, presets, calibración a dos puntos y una interfaz local con LCD `20x4`, encoder y keypad.

## Qué hace

Este proyecto controla una fuente lineal de laboratorio con:

- modo de ajuste de tensión `CV`
- modo de ajuste de corriente `CI`
- presets rápidos de tensión y corriente
- menú de configuración en pantalla
- calibración de sensado y salida por dos puntos
- guardado de factores de calibración en EEPROM
- simulación en Wokwi y entorno para hardware real

## Interfaz

La interfaz principal muestra:

- mensajes y temperatura en la línea superior
- valores instantáneos de `V` e `I`
- potencia instantánea
- campo de ingreso numérico para seteo o valor real durante calibración

Con la tecla `S` se accede a funciones contextuales:

- `S + 1..6`: memorias de presets
- `S + 0`: menú `Config.`

## Menús actuales

`Config.` incluye por ahora:

- `Limits`
- `Calibration`
- `Exit`

`Calibration` permite:

- `Cal V`
- `Cal I`
- `Load Cal`
- `Save Cal`
- `Back`

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

Versión actual en trabajo: `v1.54b beta`

Pendientes principales:

- revisión y optimización de RAM/Flash
- completar menú `Limits`
- revisar futuras protecciones `OVP/OCP`
