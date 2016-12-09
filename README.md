# Perfect_10

## Tasks:

- [ ] Create states for positions along swing
- [ ] Compute time-to-landing for person onto platform
- [ ] Process camera to determine platform velocities

## Algorithm:

- Update `x_land`, `t_land` from person position, velocity, and acceleration
- Update `v_platform` from camera sensor
- Query `x_platform` at time `t_land`
- If `x_land ~= x_plaform`, then jump

## TODO:

- Desired joint angles for swing states
