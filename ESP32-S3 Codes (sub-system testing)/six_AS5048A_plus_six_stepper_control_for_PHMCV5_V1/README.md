#six_AS5048A_plus_six_stepper_control_for_PHMCV5_V1

Code to sanity test if the controller board with TMC2209 motor driver and a single encoder attached are working.
Stepper motor drives forward and backward by 500 steps each time and repeatedly. Absolute encoder takes absolute reading at the end of each direction before stepper switch direction again.

Setup as shown:

Integrated stepper motors (Nema 17) and encoders (AS5048A)

![6DOF_jogging_and_read_encoders -gif](https://github.com/user-attachments/assets/6463ab03-2480-44d6-90ef-16be18155748)


Stepper motors Stepping and Direction pins

<img width="936" height="361" alt="image" src="https://github.com/user-attachments/assets/d4077294-bce9-491a-8778-c3610a4bae55" />

Absolute Encoders jumper settings and pins

<img width="785" height="525" alt="image" src="https://github.com/user-attachments/assets/6f43a2ad-cd37-48ae-ad7a-194b79292ede" />
