# single_AS5048A_plus_one_stepper_control_for_PHMCV5_V1

Code to sanity test if the controller board with TMC2209 motor driver and a single encoder attached are working.
Stepper motor drives forward and backward by 500 steps each time and repeatedly. Micro-stepping setting at 1/8.
Absolute encoder takes absolute reading at the end of each direction before stepper switch direction again.

Setup as shown:
<img width="633" height="473" alt="setup" src="https://github.com/user-attachments/assets/9bc0c3c0-d25e-4a8c-8ab9-5841e89ca33a" />
