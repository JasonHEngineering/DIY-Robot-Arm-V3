# Notes 

1) Along with using with TMC2209 motor drivers, I have removed R8, R15, R16, R17, R18, R19 because the R value was accidentally too high. I have shorted R8, R16, R17, and left open R15, R18, R19.

<img width="2008" height="992" alt="image" src="https://github.com/user-attachments/assets/a7317916-9da4-4d96-8672-569101f01d96" />


2) Servo is indeed GPIO 38 (J6 - pin 3), instead of the initial intent of GPIO 43 (J13 - pin 3), because the serial com line is always busy

<img width="520" height="1074" alt="image" src="https://github.com/user-attachments/assets/0eeead01-d2f2-4095-9377-a0e2af0ca67e" />

