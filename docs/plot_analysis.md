# Attempt 1
### Angle / RPM
- Angle increases instantly from 0 to 300+ degrees
- RPM increases from 0 to 1000 rpm which is 104.71975512 rpm / 16.67 Hz

### Switches
It seems like not even 1 electrical cycle is passed, uh and wl are not even switched.

# TODO:
- Add torque plot
- Figure out why in 10 ms it just instantly moves 300 degrees and gets to a fixed position, while the control PWM frequency is 16e3, so the electrical period is supposed to be 6 / 16e3 = 0.000375 = 3.75 ms, so we're supposed to have passed 3 periods but nothing like that seeemed to have happened.
- Add a simulation with slower acceleration, think of how the simulation can be speed up, perhaps do something with C instead of python since it seems to be too slow.
