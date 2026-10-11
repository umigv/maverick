# mppi

MPPI controller over a unicycle model. Experimental: select it with `algorithm:=mppi`; `differential_drive` remains the default.

At each control step, the controller samples `num_samples` control sequences around its current plan (the nominal sequence), rolls each one out `horizon_steps` steps of `timestep_s` through the unicycle model, and scores the resulting trajectories. The new plan is the average of the samples, weighted by `exp(-(cost - min cost) / temperature)`. Its first control is published, and the plan is shifted forward by one control period to warm-start the next step.

The cost combines cross-track error, heading error, deviation from a reference speed, and changes in velocity, minus a reward for progress along the path. The reference speed is `target_speed_mps`, tapered so the robot can decelerate to a stop at the goal:

```
v_ref = min(target_speed_mps, sqrt(2 * max_linear_accel_mps2 * (remaining_arc - goal_tolerance_m)))
```

Each rollout state is matched only to the part of the path it could have reached by then.

**Noise reuse:** (Because drawing fresh noises cause a very heavy jitter problem) With `reuse_noise`, the samples are kept between steps and only `noise_refresh_pairs_per_call` of them are replaced each step. Samples come in mirrored pairs, so their average stays zero.

**Speed resync:** If the measured speed is more than `speed_resync_threshold_mps` below the last command (for example while the ramp speed cap is active, or another source has control), planning restarts from the measured speed instead.
