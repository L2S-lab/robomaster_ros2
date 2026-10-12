# Manual checkpoint checks (not run)

- [ ] C1 Discovery: discover one robot, then two robots; verify unique devices and bounded scan timeout.
- [ ] C2 Arm: verify X/Z directions, 20 Hz output, +/-0.20 m/s clamp and 0.03 m step limit; NaN/Inf, disarm, close and 0.25 s silence stop new increments. Tracked arm actions take priority; rearming never replays cached commands.
