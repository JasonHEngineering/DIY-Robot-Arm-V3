# Servo mechnism basic sanity check

Code to sanity test if the Servo mechnism is working. 
PWM pin used from ESP32-S3 was GPIO20 - take note that this is usually used as USB D+. 
Servo in this case was a MG996R, wired with an independent 5V source (I was using a Mini 560 PRO).

Wiring colours for MG996R:

Yellow - PWM signal 

Red    - 5 to 6 V

Brown  - Ground


Preview:

![servo gif](https://github.com/user-attachments/assets/1622c2fe-5fe6-45fc-98b1-31c3be341b0c)
