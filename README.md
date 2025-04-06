# Pico_4Servo_4Frog_2Crossover_Wifi
This program;-
- is an extension to Pico_4Servo_4Frog_WiFi and adds the ability of creating crossovers from pairs of servos.
- uses the files StateMachine.h and StateMachine.cpp to replace the nested switch/case code of Pico_4Servo_4Frog_Wifi.
- runs on the same hardware as Pico_4Servo_4Frog_WiFi.

Servos
- Each servo has 3 positions whose factory reset positions are;-
    - position 1 (Thrown) - 80 degrees
    - position 2 (Mid) - 90 degrees
    - position 3 (Closed) - 100 degrees
- This allows only the smallest of movements which can be adjusted when connected to a turnout.
- Each position has 1 consumed event and 2 produced events as below;-
    - a consumed event which starts the servo moving to that position.
    - a produced event which is sent when the servo reaches that position.
    - a produced event which is sent when the servo leaves that position.
- Outputs provide for frog switching. JMRI can be used to configure the frog switching outputs so that the frog is switched according to the servo position reached.
- The mid point can be used to centre the servo for installation.
- When the module starts it sends out the leaving event for all positions for all servos and sets all servos to
their mid position.

Crossovers
- The four servos can opearte as two pairs of crossover turnouts. Each crossover has its own events to move to its three positions. Also, each crossover has its own leaving and reached events. When a crossover receives an event which causes it to leave its current position the crossover's first servo will start moving and the crossover's leaving event will be sent. Then the second servo starts moving. When both servos reach their target positions the crossover's reached event will be sent.
- When a crossover's two servos are moving they still send their individual events independently as normal (these can be ignored by JMRI).
- When the module starts it sends out the leaving event for all positions for all crossovers.

