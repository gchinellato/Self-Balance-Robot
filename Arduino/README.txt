# Arduino Nano

Instantiate Motors objects
Instantiate Encoder objetcs
Instantiate PID objects
Instantiate IMU objects (Accel, Gyro, Magno, Barom)

setup()
{
    Configure serial
    Configure interrup pins
    Set configuration
}

loop()
{
    Read serial trace
    Read sensors
    Sensor fusion (Euler angles)   
    Update speeds
    Compute speed PID
    Check angle
    Define PID mode
    Compute angle PID
    Motor set speed
    Send serial trace  
}

