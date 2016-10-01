setup()
{
    Configure serial
    Configure i2c
    Configure pwm pins
    Configure interrup pins
    Configure output pins
    Init IMU Module
    Init Motion Module
    Init PID Module
}

loop()
{
    read Serial
    read Sensors
    Compass Heading
    Matrix update
    Normalize
    Drift correction
    Euler Angles
    Update speeds
    PID compute
    Motor set speed
    send Serial  
}

