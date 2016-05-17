L3G4200D-Python
===============

L3G4200D Gyro interface in python, using I2C.

Currently, this code opens port i2c-1 in /dev and imports raw data(signed int) of each gyro axis.
you must have python installed "apt-get install python"
in command line do "python gyro.py" to stream raw axis data from the gyro. the readings are noisy, and that's normal.

you should get something like this:

        -1        -14          6
         4        -15          5
        -1        -15         10
        -4        -19          4
        -2        -15         11
         2        -19          7
         1        -17         10
         8        -15         10
         5        -16          7
         2        -17          6
         0        -18         10
        -2        -16         14

you can type "python gyro.py > data.tsv", then ctrl C to stop. take data.tsv, open it in MS Excel
or equivelant, and plot the data to visualise the gyro movements.
