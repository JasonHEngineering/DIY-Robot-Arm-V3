# Servo mechanism basic sanity check

Code to sanity test if the Servo mechanism is working. 
PWM pin used from ESP32-S3 is now GPIO38 (although it was originally with GPIO 43, which is not suitable due to Serial com)
Servo in this case was a MG996R, wired with an independent 5V source (I was using a Mini 560 PRO).

<!-- wp:image {"id":4318,"sizeSlug":"large","linkDestination":"none"} -->
<figure class="wp-block-image size-large"><img src="https://jashuang1983.wordpress.com/wp-content/uploads/2026/07/image-6.png?w=734" alt="" class="wp-image-4318"/></figure>
<!-- /wp:image -->

Wiring colours for MG996R:

Yellow - PWM signal 

Red    - 5 to 6 V

Brown  - Ground


Preview:

![servo gif](https://github.com/user-attachments/assets/1622c2fe-5fe6-45fc-98b1-31c3be341b0c)
