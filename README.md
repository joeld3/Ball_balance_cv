# Ball_balance_cv

This program is part of a project that uses computer vision to balance a ping-pong ball in the middle of a hinged track. The program opens a webcam which looks down upon the physical track and determines the error between the center of the track and the location of a ping-pong ball within the track.  This error is fed into a PID controller, which generates a control output.  This output is an integer which is used in the generation of a PWM signal to drive a servo. The servo is connected to the track with a linkage arm, which allows it to adjust the angle of the track.

Since desktop PCs typically do not have GPIO pins, the control output is sent over USB to a TM4C123GH6PM microcontroller. This microcontroller is programmed with embedded C to implement the PWM and drive the servo motor.

The embedded C program currently utilizes functions contained in Texas Instruments' TivaWare SDK, which is subject to export administration regulations.  Due to this, the C program will not be uploaded to this repository.
