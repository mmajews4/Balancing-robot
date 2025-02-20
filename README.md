# Balancing robot

![balancing robot image](/images/balancing_robot.jpg)

This is the reposiotry of my journey to learning control engenering by making balancig robot project

Steering (you can add macro buttons for some of this controlls):
- Startup mode:

    - by default robot needs to be placed vertically, on its frame, on a stable ground for calibration. Then you can power it up.
    - r - go to run mode
    - q - go to settings mode

- Run mode:

    Console shows, and updates this specs of the robot:
    
    <code>P:[proportional value]   I:[integral value]   D:[derivative value]
    Angle: []
    Steer: []    Turn: []
    Throttle: []   MotSpeed: []</code>
    - t - terminate run mode and reset robot
    - q - go to settings mode
    - w, s - increment target angle by "steer factor" for robot to drive foreward or backward
    - a, d - turn
    - 1-9 - set specific target angle (num*steer_factor)

- Settings mode:

    Console shows, and updates this specs of the robot:

    <code>P:[proportional gain]   I:[integral gain]   D:[derivative gain]
    Angle: []
    Steer: []
    Gyro: [angle calculated with gyroscope]  Accel: [angle calculated with acclerometer]</code>
    - r - go to run mode
    - t - terminate settings mode and reset robot
    - q - go to settings mode
    - c - calibrate angle (needs about 10s to be done)
    - p[num] - set proportional gain   
    - i[num] - set integral gain   
    - d[num] - set derivative gain
    - s[num] - set value of setter factor
    - a[num] - set max motor acceleration
    - m[num] - set compensation of angle
    - 1-9 - set specific target angle (num*steer_factor)