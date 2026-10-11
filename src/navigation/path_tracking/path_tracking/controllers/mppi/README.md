# mppi

Model Predictive Path Integral (MPPI) path tracking controller over a differential-drive (unicycle) model. Experimental: `differential_drive` remains the default controller. Select with the parameter `algorithm:=mppi`.

## Algorithm

Each control step:

1. Project the robot onto the path (same forward-only `Path2d.project` and goal handling as `differential_drive`).
2. Sample `num_samples` control sequences of `horizon_steps` steps around the nominal sequence, with Gaussian noise on `[v, omega]` (standard deviations `linear_noise_std_mps`, `angular_noise_std_radps`). One sample is the unperturbed nominal sequence.
3. Roll every sample out through the unicycle model (`x += v cos(yaw) dt`, `y += v sin(yaw) dt`, `yaw += omega dt`, `dt = timestep_s`), vectorized over samples.
4. Score each rollout (see Cost) and weight samples by `exp(-(S - min S) / temperature)`. Subtracting the minimum keeps the exponent numerically stable.
5. The new nominal sequence is the weighted average of the sampled sequences. Its first control, limited against the previous command over one control period, is published as `linear.x` / `angular.z`.
6. Warm start: the nominal sequence is shifted forward by one control period (linear interpolation, last control held).

The node runs at `control_period_s` (10 ms by default) and re-optimizes every step; `timestep_s` (0.1 s) only sets the rollout resolution.

## Constraints

Enforced by projection on every sampled control, in the rollouts and again on the output command:

- `0 <= v <= max_linear_speed_mps` (no reverse) and `|omega| <= max_angular_speed_radps`
- `|dv| <= max_linear_accel_mps2 * dt` and `|domega| <= max_angular_accel_radps2 * dt` relative to the previous control (`dt = timestep_s` in rollouts, `control_period_s` on the output). Linear and angular changes are limited separately, so one wheel can be asked for up to 6 m/s^2 (twice the ODrive ramp, which then limits it). A combined per-wheel limit (`|dv| + |domega| * track_width_m / 2 <= 3 m/s^2 * dt`) was tested and reverted: in simulation, whose robot follows commands instantly, it made tracking worse (cross-track RMS 0.125 -> 0.156 m, time to goal 7.94 -> 9.08 s over 12 trials each); with a ramp-limited robot model offline the cost was small (0.139 -> 0.145 m). Re-evaluate on hardware.
- Wheel speeds `|v -/+ omega * track_width_m / 2| <= max_wheel_speed_mps`. Speed is capped first so a rate-feasible `omega` always exists.

All samples start from the same previous command and the constraints are convex, so the weighted average is feasible as well. The single zero command issued at the goal is the only exception to the rate limits, matching the other controllers.

Default limits follow `odrive_driver`: `track_width_m=0.764`, `accel_limit_mps2=3.0` (per wheel), and a wheel limit of 2.0 m/s below the 2.2 m/s hard trip (`vel_limit_mps`). `max_angular_accel_radps2=7.85` is the per-wheel limit expressed as a pure rotation (`2 * 3.0 / 0.764`). Note the URDF uses a different track width (0.711 m). At 2.0 m/s the wheel limit leaves no room to turn, so the robot slows down in curves.

## Command Lag (removed)

A first-order lag between the optimized control and the published command, modelled in the rollouts, was tested and removed. A 50 ms lag halved wheel acceleration chatter in simulation, but reused noise (below) removes most of it without delaying the command, and a 100 ms lag had occasional invalid trials.

## Noise Reuse (default)

The command chatter comes from drawing fresh exploration noise on every 10 ms update: the weighted average of the samples jumps from call to call by about as much as the per-call rate limit (0.03 m/s), so the published command bounces between its rate limits. With `reuse_noise` (default on) the controller keeps the sample set between updates and replaces `noise_refresh_pairs_per_call` samples (default 1) and their mirrors per call, so consecutive updates see almost the same samples and the whole set still renews every `num_samples / 2` calls (2.5 s). The second half of the samples mirrors the first, so the samples sum to exactly zero and a kept set cannot bias the update. The noise standard deviations are halved (`linear_noise_std_mps=0.2`, `angular_noise_std_radps=0.4`), which keeps each replacement's effect on the update small. Set `reuse_noise:=false`, `linear_noise_std_mps:=0.4`, `angular_noise_std_radps:=0.8` for the previous behavior.

Simulation, test3 (goal 7.5, 1.5), interleaved trials, wheel metrics after a model of the ODrive 3 m/s^2 ramp (28 trials each, after the occupancy grid timing fix below):

|                                              | fresh noise (previous) | reused + halved noise (default) |
| -------------------------------------------- | ---------------------- | ------------------------------- |
| Valid trials                                 | 28/28                  | 28/28                           |
| Wheel acceleration reversals/s (> 0.5 m/s^2) | 34.8                   | 2.8                             |
| Wheel acceleration reversals/s (> 1.5 m/s^2) | 33.7                   | 2.1                             |
| Wheel acceleration swing RMS (m/s^2)         | 2.05                   | 1.07                            |
| Time at the ramp limit                       | 62%                    | 27%                             |
| Cross-track RMS / max (m)                    | 0.143 / 0.28           | 0.135 / 0.33                    |
| Heading error RMS (rad)                      | 0.250                  | 0.247                           |
| Time to goal (s)                             | 7.62                   | 8.27                            |

A further 12-trial sweep (also 1, 2 and 4 pairs per call), 8 trials with the whole simulation pinned to 4 CPU threads, and 8 trials on the test2 course were all valid for both settings. The costs are about 8% more time to goal and a larger worst-case deviation (up to 0.46 m vs 0.32-0.36 m on test3). More pairs per call renew the set faster and reduce run-to-run variation at the cost of some chatter (4 pairs: 3.4 large reversals/s).

Before the grid timing fix, reused noise had 4 invalid trials out of 28 (0 for fresh noise). They were traced to `occupancy_grid_transform` placing each grid with the latest transform instead of the one at capture time: obstacles in the planner's grid were shifted by 28-37 cm (median, up to 91 cm) 3 m ahead, and in narrow configurations the planner fell back to a path ending at an obstacle face. Replaying the recorded failures offline reproduced the bad paths within a few centimeters and showed that correctly placed grids give the safe path. Reused noise made runs vary more from one another (the sample set persists for 2.5 s), so they reached those configurations more often. With grids placed at their capture time, no trial of either setting received such a path.

Things that did not help: stronger rate penalties (no effect at 10x; the robot never starts at 100x), a jerk penalty (same at strong weights, no gain at weak ones), a penalty on the first control's deviation from the previous plan (worse tracking), a higher temperature (worse tracking), noise that only partly changes between calls (most of the jitter remains; the weighted update is sensitive to any per-call change of the samples), and reusing only half of the samples (same). Keeping the whole sample set frozen removes the jitter but makes tracking depend on the particular samples drawn.

## Cost

The path is resampled to `path_resolution_m` and each rollout state is projected onto the segments within reach of the horizon. Running cost per step (multiplied by `timestep_s`):

- `cross_track_weight * d^2`: squared distance to the path
- `heading_weight * wrap(yaw - path_heading)^2`: heading error, wrapped to [-pi, pi]
- `speed_weight * (v - v_ref)^2`, with `v_ref = min(target_speed_mps, sqrt(2 * max_linear_accel_mps2 * max(remaining_arc - goal_tolerance_m, 0)))` so the reference only drops to zero where the robot stops, `goal_tolerance_m` before the end of the path (it arrives at about 0.2 m/s instead of 0.5 m/s); standing still is never cheap before the goal
- `linear_rate_weight * dv^2 + angular_rate_weight * domega^2`: command smoothness

Terminal cost: `terminal_cross_track_weight * d_T^2 - progress_weight * (arc length progressed)`.

Each rollout state may only be matched to the part of the path it could have reached: segments starting no further than the robot's own path position plus the distance that rollout has traveled plus `PROJECTION_ARC_MARGIN_M` (0.5 m). Without this, a rollout that cuts across to a nearby later part of the path (the far leg of a hairpin around an obstacle) is matched to it, scores a small cross-track error and a large progress reward, and MPPI skips the turn. In an offline test with a 3 m out, 180 degree (0.5 m radius), 3 m back path, the unbounded projection cut across at x = 1.58 instead of turning at x = 3.5.

Obstacles are not considered; the controller tracks the planner's path.

## Path Updates

`set_path` resets path progress and caches and refills the nominal sequence with the last applied command (not zeros), so frequent replanning does not cause rate-limit violations. After the goal is reached, all state is reset to zero.

## Measured `speed`

Rate limits and rollouts start from the last published command, which assumes that command was applied. When it was not (the node caps `linear.x` while `in_ramp_approach` is set, twist_mux hands control to recovery or teleop, or the loop is skipped after `mission_complete`), MPPI would plan from a speed the robot is not moving at, and the published speed would jump when it regains control.

The `speed` argument (measured odometry speed) guards against this. If it is lower than the last published command by more than `speed_resync_threshold_mps`, the stored command is reset to the measured speed before optimizing, so the published speed can lead the measured speed by at most the threshold plus one rate-limit step. Only downward differences are resynchronized: the measured speed is a magnitude, so it cannot distinguish reversing from driving forward, and lowering the stored speed always keeps it feasible.

The threshold must exceed the drive's normal speed tracking lag. In simulation the last command led the measured speed by at most 0.095 m/s (8 trials, about 5,300 calls while moving, no resync fired), and the default is 0.3 m/s. On hardware, check this before relying on the default: if the drive normally lags its command by more than the threshold (for example a velocity loop without an integrator, under load), MPPI will repeatedly resynchronize and cannot accelerate past the measured speed plus the threshold. A single low odometry sample also triggers a resync, which briefly lowers the command.

## Compute Cost

Latency scales roughly with `num_samples * horizon_steps * (segments in reach)`. Measure on the target computer with:

```
pixi run python3 src/navigation/path_tracking/benchmark/benchmark_controller.py --algorithm mppi
```
