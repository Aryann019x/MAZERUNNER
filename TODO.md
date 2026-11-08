# TODO

- [ ] Reduce turning time / prevent rapid re-turning loops that cause the robot to miss the freeway
- [ ] Implement a “turn cooldown” or “turn duration” using non-blocking timing (no long delay inside turning)
- [ ] Replace `delay(turn90Time)` in `turnLeft/turnRight` with a state-based timed turn so sensor decisions have time to update
- [ ] Add a safety: while turning, ignore obstacle triggers until the turn duration elapses
- [ ] Make one long recording of the wall follower and train on that instead of synthetic data
- [ ] Try a smaller laptop model or streaming ollama so the drive loop is faster than 2 s
- [ ] Log neural page decisions over serial for tuning
- [ ] Build/verify in PlatformIO (compile)
