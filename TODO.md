# TODO

- [ ] Flash the motor-side fix and check that a straight command really goes straight
- [ ] Test the neural page in a real maze (the simulator is only a guide)
- [ ] Check the learned trim over a full battery charge (motor speed changes as voltage sags)
- [ ] Log neural page decisions over serial for tuning
- [ ] Make one long recording of the wall follower and train on that instead of synthetic data
- [ ] Try a smaller laptop model or streaming ollama so the drive loop is faster than 2 s
- [ ] Run a longer CEM sweep and compare the result on the floor
- [ ] Add encoders (or trust the gyro more) so pose dead reckoning survives wheel slip
