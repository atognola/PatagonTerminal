Firmware para dispositivos Shangri-La que implementa una terminal para debug del SIM900

Instalación:
- Instalar Atmel Studio 6: http://www.atmel.com/tools/atmelstudio.aspx
(instalar al directorio default)
- Instalar CodeSourcery G++: 
Instalar en C:\Program Files (x86)\CodeSourcery\Sourcery G++ Lite
(es el default).
Al final de la instalación, seleccionar "Do not modify PATH".

- En Atmel Studio, ir a Tools - Extension Manager y actualizar el Atmel Gallery y Atmel Software Framework a la versión 3.5.1.

- En Atmel Studio, ir a Tools - Options. Seleccionar Toolchain - Flavour Configuration.
- Seleccionar, para Armel ARM 32-bit, la opción Add Flavour, y poner los siguientes parámetros:
Flavour name: CodeSourcery
Toolchain Path: C:\Program Files (x86)\CodeSourcery\Sourcery G++ Lite\bin
Make Path: C:\Program Files (x86)\Atmel\Atmel Studio 6.0\make
- Clickear "Set as Default"

- Copiar la carpeta:
C:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMSupportFiles
a:
C:\Program Files (x86)\CodeSourcery

- Clonar el proyecto usando el cliente de GitHub para Windows.