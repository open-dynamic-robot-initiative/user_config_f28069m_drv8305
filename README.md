MotorWare Configuration Files
=============================

This repository contains the global configuration files that are used by all
our MotorWare projects.  It mainly contains the motor and board parameters that
are specific to the hardware that is used.

When changing configuration for a new hardware setup, please create a new
branch with a meaningful name (something like "leg_prototype_1").  Do *not* use
the master branch.

This way it is always clear which configuration is currently used and danger of
messing up with other projects is reduced.


About the files
---------------

Single motor applications use the files `user.h`, `user_j1.h` and `user_j5.h`.
The first one configures the microcontroller and decides which of the two
motors is used by the application.  The other two contain the motor specific
configuration for J1 and J5 respectively.

Dual motor applications use the files `user1.h`, `user2.h`, `user_mtr_on_j1.h`
and `user_mtr_on_j5.h`.  The first two correspond to the `user.h` of the single
motor case (I don't now why there are two files as most of the configuration is
redundant and probably only the values of `user1.h` are actually used...).  The
last two contain the configurations for J1 and J5.

Note: In `user_mtr_on_j5.h` the names of all defines have to be suffixed with
`_2`!


This design of the configuration files is very confusing and leads to lots of
code duplication (motor parameters have to be copied to all four `*_j?.h`
files).  We should change this to a better structure...


For more information on the configuration files, see the Confluence Wiki:
https://atlas.is.localnet/confluence/display/AMDW/InstaSPIN+Configuration+Files


License
-------

BSD 3-Clause License

Copyright (c) 2015, Texas Instruments Incorporated
Copyright (c) 2019, Max Planck Gesellschaft, New York University
