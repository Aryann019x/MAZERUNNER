# TODO

- [ ] Reduce turning time / prevent rapid re-turning loops that cause the robot to miss the freeway
- [ ] Implement a “turn cooldown” or “turn duration” using non-blocking timing (no long delay inside turning)
- [ ] Replace `delay(turn90Time)` in `turnLeft/turnRight` with a state-based timed turn so sensor decisions have time to update
- [ ] Add a safety: while turning, ignore obstacle triggers until the turn duration elapses
- [ ] Build/verify in PlatformIO (compile)
