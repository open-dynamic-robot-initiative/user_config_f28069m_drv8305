**********************************************************
Welcome to ODRI Motor Board Configuration's documentation!
**********************************************************

.. toctree::
   :maxdepth: 1
   :hidden:

   motor_id


The user_config_f28069m_drv8305_ repository contains the firmware configuration
files for the ODRI project.  They are based on the config files included in the
MotorWare_ examples and can be used for those.  Likewise they are used by our
:doc:`firmware <mw_dual_motor_torque_ctrl:index>` (both CAN and SPI versions).

.. note::

    The structure of these config files is not ideal.  Configuration is
    duplicated for the single-motor and dual-motor applications and also some
    global, motor-indepented values (like CPU frequency) are duplicated in the
    config files for the second motor but those duplicates are not used
    anywhere.

    This is because the files are based on the configuration files of the
    MotorWare example projects and probably were not meant to be used like this
    in production.  However, we keep this structure to preserve compatibility
    with those example projects.


About the Different Files
=========================

There are different files for single motor and dual motor projects. For each
case there are again separate files for the motors connected to J1 and J5. Lots
of the content of these files is the same so there is quite some copy&past
mess.


Single Motor Applications
-------------------------

.. list-table::
    :widths: 1 3
    :header-rows: 1

    * - Filename
      - Purpose
    * - user.h
      - Main configuration file. Configures the microcontroller and which of
        the motors is used (J1 or J5).
    * - user_j1.h
      - Configuration specific for the motor and inverter board on J1.
    * - user_j5.h
      - Configuration specific for the motor and inverter board on J5.

Note that user_j1.h and user_j5.h have exactly the same structure.


Dual Motor Applications
-----------------------

.. list-table::
    :widths: 1 3
    :header-rows: 1

    * - Filename
      - Purpose
    * - user1.h
      - Corresponds to user.h for single motor applications and includes config for J1
    * - user2.h
      - Same as user1.h but includes config for J5.\ :sup:`1`
    * - user_mtr_on_j1.h
      - Same as user_j1.h.
    * - user_mtr_on_j5.h
      - Same as user_j5.h but all defines are suffixed with "_2".

:sup:`1` Note that ``user2.h`` also duplicates all the global settings of
``user1.h`` (e.g. CPU frequency), which does not really make sense.  Those
settings are most likely not used anywhere, as the ones from ``user1.h`` are
used.

.. note::
    Note that configuration for J1 is duplicated between user_j1.h and
    user_mtr_on_j1.h, so make sure to apply relevant changes to both files. The
    same holds for user_j5.h and user_on_mtr_j5.h (where the latter has suffix
    "_2" in all defines).


Select Motor in Single-Motor-Application
========================================

When running a single motor application (i.e. one that uses ``user.h`` instead
of ``user1.h, user2.h``, it has to be defined in the configuration file which
of the two slots (J1 or J5) shall be used. This is done in ``user.h``. At the
top of the file there is a line

.. code-block:: C

   #define J1  // or J5

Set this to J1 or J5 to use the corresponding motor.


Configure System Frequencies
============================

As explained in
:doc:`mw_dual_motor_torque_ctrl:program_structure_and_frequencies`, different
parts of the program run at different frequencies. This is accomplished by
defining a base frequency and a set of "decimation factors". This is done in
the ``*_j1/5.h`` configuration files, so different frequencies can be set for
the two motors.

The base frequency is the PWM, which is defined by ``USER_PWM_FREQ_kHz`` in the
section "CLOCKS & TIMERS". For a single motor, this can be set to something
like 45. For dual motor applications set it to a lower value as there has to be
enough CPU time left for the second motor. It may also be good to set slightly
different values for the two motors so that the controllers are not always
triggered at the same time (the example projects use 20 and 18, which seems to
work well).

The decimation factors are defined in the section "DECIMATION" as follows:

.. code-block:: C

   #define USER_NUM_X_TICKS_PER_Y_TICK n

This means that every n-th time X is executed, Y will be run. If X runs with
frequency :math:`f_X` the resulting frequency for Y will be

.. math:: f_Y = \frac{f_X}{\text{USER_NUM_X_TICKS_PER_Y_TICK}}


.. admonition:: Example

    If ``USER_PWM_FREQ_kHz = 20``, ``USER_PWM_TICKS_PER_ISR_TICK = 2``, and
    ``USER_ISR_TICKS_PER_SPEED_TICK = 10``, the ISR is executed every second
    PWM cycle and every 10-th time the ISR is run, it will run the speed
    controller. This means ``f_ISR = 10kHz`` and ``f_controller = 1kHz``.

Below is a complete list of all decimation factors that are defined:

.. list-table::
    :widths: 1 3
    :header-rows: 1

    * - Name
      - Purpose
    * - USER_NUM_ISR_TICKS_PER_CTRL_TICK
      - Defines the frequency of the control loop.

        ⚠️  This is not implemented correctly. **Always set it to 1!**
    * - USER_NUM_CTRL_TICKS_PER_CURRENT_TICK
      - Defines the frequency of the low-level current controller.

        ℹ️  This is actually the not used! The current controller runs at
        ISR frequency.
    * - USER_NUM_CTRL_TICKS_PER_EST_TICK
      - Defines the frequency of the FAST estimator.

        ℹ️  This is not used in our code, as we do not use FAST.
    * - **USER_NUM_CTRL_TICKS_PER_POSCONV_TICK**
      - Defines the frequency of the SpinTAC PositionConverter module
        (responsible for providing motor position and speed).

        ⚠️  Must be at least three times the frequency of highest expected
        electrical velocity, otherwise velocity will suffer from an overflow!
    * - **USER_NUM_CTRL_TICKS_PER_SPEED_TICK**
      - Defines the frequency of the high level controller (which, depending on
        the firmware that is used, does not necessarily have to be a speed
        controller).
    * - USER_NUM_CTRL_TICKS_PER_TRAJ_TICK
      - This is not used by our code.

For the dual_motor_torque_ctrl project (which is the one that is used on the
board when it is controlled via CAN), only the bold factors should be modified.

Configure Motor Parameters
==========================

Adding new motors to the configuration is handled in :doc:`motor_id`.

Further Reading
===============

Checkout the Texas Instruments' InstaSPIN-FOC/-MOTION User's Guide for more
information on the various parameters and their effects.


.. _user_config_f28069m_drv8305: https://github.com/open-dynamic-robot-initiative/user_config_f28069m_drv8305
.. _MotorWare: https://www.ti.com/tool/MOTORWARE
