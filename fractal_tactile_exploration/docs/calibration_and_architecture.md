# FRACTAL Omega Mapping

`omega_explorer_node` is the first implementation of the framework in `source_material/algorithm.tex`.
It subscribes to `/paxini/L5325_omega/tactile_sensor` and `/franka_robot_state_broadcaster/robot_state`.
Every accepted guarded contact publishes `~/map_sample`, preserving the contact `XYZ`, `RPY`, Franka force, fused contact metric, and Omega-derived compliance. A Franka hard-force violation publishes `~/emergency_retract_target`, displaced by `safety.retract_distance` along `tactile_map` +Z.

## Algorithm

The static map is two independent Gaussian Processes over the tactile-map plane: geometry and compliance.
For each contact sample, the node updates both posteriors and evaluates every configured XY candidate:

$$
c = w_F\lVert\mathbf{F}_{\mathrm{Franka}}\rVert + w_T\sum_i\lVert\mathbf{f}_{i,\mathrm{Omega}}\rVert
$$

$$
\operatorname{score}(x,y) = \lambda_1(\sigma_g^2 + \sigma_m^2) + \lambda_2\min\left(\frac{\operatorname{flatness}}{\operatorname{target\_flatness}}, \frac{\operatorname{firmness}}{\operatorname{target\_firmness}}\right)
$$

The selected approach pose is published at `~/next_omega_target`. A MoveIt execution adapter should convert that target to a collision-checked trajectory and publish its joint references to `/fractal_joint_reference_impedance_controller/reference`. It is intentionally not included until the pad-aware MoveIt configuration is available.

## Tunable Framework Modules

All first-stage entries are parameters in `config/omega_explorer.yaml`:

| Framework module | Parameters |
| --- | --- |
| Observation space | Omega full-taxel topic and `sensor.measurement_stddev` |
| Action space | `candidate_x`, `candidate_y`, `approach_height`, `nominal_yaw` |
| Material and geometry GP | `geometry.*`, `material.*` |
| Interaction process | Fused contact metric weights and thresholds |
| Sensor model | Taxel-force sum and Franka external-force estimate |
| Affordance/reward | `affordance.target_flatness`, `affordance.target_firmness`, weights |
| Safety/cost | `safety.*`; the execution adapter must reject poses outside RPY/workspace limits and retract along map +Z |
| Elite extension | Add an Elite sensor strategy that supplies samples and candidate actions through the same map interface |

## Translation-Only `tactile_map` Calibration

Keep the table level relative to `panda_link0` and hold the Omega orientation fixed. Use a flat calibration plate with known points. At each point, make a low-force contact and record the contact position from `FrankaState.o_t_ee` after applying the known pad-to-EE offset. The translation is:

$$
{}^{\mathrm{panda\_link0}}\mathbf{t}_{\mathrm{tactile\_map}} = \frac{1}{N}\sum_i\left(\mathbf{p}^{\mathrm{panda\_link0}}_i - \mathbf{p}^{\mathrm{tactile\_map}}_i\right)
$$

Set that three-element result as `map_translation_xyz` in both the explorer configuration and static TF publisher. Verify the residual at multiple plate points before enabling real-arm descent. Translation-only calibration is invalid when the map plane is tilted; add rotation calibration before operating in that case.