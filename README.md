# MS_Soldering_Station
Modular Soldering Station project by TA1AYH & TA2PLT

Hi, with this module you can;
- select the target temperature of solder (auto PWM),
- select the sleep time period from menu,
- select sleep PWM limit (when auto PWM drops below this value, sleep time counter starts, when it drops 0:0 the soldering station heater starts to sleep)
- control sleep function. it starts to work when auto pwm drops to below the limit pwm (after heater temperature reach target temperature on the auto PWM),
- control IR sensivity to wake up the heater to reach the target temperature or reset the sleep timer,
- select supply voltage input (18-20-24v DC), ********* do not use more than 24v DC when using single DC input to the board *************
- set the max user temperature ( if this temperature reach with your temerature setting, it automatically sets -5 degrees celcius below the max user temperature),
- set firmware Max protection temperature with the arduino code( if the system accidently reach this temperature, target temperature will set the 0 to protect the heater)
- selecet the user interface in two types of screen (graphic and windowed modes)
- auto reset function allows you to reset the sleep timer (when you do not use solder it starts to count down the timer, if you start use solder, auto PWM sens and increase the PWM to keep target temperature, and it resets the timer)

For auto reset fuction i wrote own temperature control logic (standart PID temperature control codes for arduino not allowing this function).  

enjoy the project, 
73's


![IMG-20230410-WA0005](https://user-images.githubusercontent.com/5972349/231213541-9bad1236-5fbf-4eef-83b8-4069e54a4166.jpg)
![IMG-20230410-WA0009](https://user-images.githubusercontent.com/5972349/231213575-04435368-dbc2-4384-8648-97f61054a115.jpg)
![IMG-20230410-WA0015](https://user-images.githubusercontent.com/5972349/231213579-169bbc5b-d2eb-49c0-aa99-ee1b6f3b9a82.jpg)
![IMG-20230410-WA0017](https://user-images.githubusercontent.com/5972349/231213583-0682f64e-5217-43bf-9a5b-40b75f03f798.jpg)
![IMG-20230410-WA0004](https://user-images.githubusercontent.com/5972349/231213611-61cbbc17-bd97-4a18-8f9f-c3a0d41e98e4.jpg)
![3d_1](https://user-images.githubusercontent.com/5972349/231213615-fa62fd89-7f5e-4682-a1b3-cde07f5f5563.jpg)
![Clipboard01](https://user-images.githubusercontent.com/5972349/231213622-91b12676-0079-4c93-9830-45f7a77f7545.jpg)
![Clipboard02](https://user-images.githubusercontent.com/5972349/231213629-bc424112-4dbd-4e7d-a7b6-19740503c45a.jpg)
