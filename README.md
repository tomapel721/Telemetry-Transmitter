## :book: Long story short
The transmitter part of telemetry system which was implemented inside a Formula Student EV.

## :computer: How it works - technologies
PCB Design: **KiCad**

Project source code: **C with usage of HAL libraries**
Board used: **STM32L432KC**

## :file_folder: Repository structure
In main directory, you can find 2 folders:
- directory **PCBDesign** contains project of printed circuit used as HW part of the telemetry transmitter
- directory **SourceCode** contains the SW part of the project
<img width="641" height="103" alt="obraz" src="https://github.com/user-attachments/assets/7a8e9b65-bb20-4f1f-b651-a3446940d4f5" />

## :scroll: How to use my code
Usage of the code can be hard to do, but in case you want to make your own racing car, here is the brief manual:
1. Print the PCB using the _GERBER_ files
2. Solder the board elements
3. Connect soldered electronics to PC
4. Flash the STM32 using ST-LINK debugger (pins where to connect are marked on the PCB schema)
5. Enjoy;)




