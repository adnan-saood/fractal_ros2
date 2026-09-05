# FRACTAL Omega Mapping

`omega_explorer_node` is the first implementation of the framework in `source_material/algorithm.tex`.
It subscribes to `/paxini/L5325_omega/tactile_sensor` and `/franka_robot_state_broadcaster/robot_state`.
The sensor stream is continuous, but a map sample represents one discrete probing event: the first guarded reading when the fused contact metric rises above `safety.contact_threshold`. Further readings in the same contact episode are ignored until the metric falls below `safety.contact_release_threshold`. Each accepted probe publishes `~/map_sample`, preserving the contact `XYZ`, `RPY`, Franka force, fused contact metric, and Omega-derived compliance. A Franka hard-force violation publishes `~/emergency_retract_target`, displaced by `safety.retract_distance` along `tactile_map` +Z.

## Algorithm

The belief contains geometry and material Gaussian Processes, local submaps, two latent interaction regimes (`static` and `elastic`), and explicit kinematic, measurement, interaction-prior, and spatial-consistency factor records. For each discrete probe sample, the belief is updated and the policy optimizes the next continuous action; it does not treat each sensor frame as a new probe or evaluate a fixed XY candidate list:

$$
c = w_F\lVert\mathbf{F}_{\mathrm{Franka}}\rVert + w_T\sum_i\lVert\mathbf{f}_{i,\mathrm{Omega}}\rVert
$$

$$
\operatorname{score}(x,y) = \lambda_1(\sigma_g^2 + \sigma_m^2) + \lambda_2\min\left(\frac{\operatorname{flatness}}{\operatorname{target\_flatness}}, \frac{\operatorname{firmness}}{\operatorname{target\_firmness}}\right)
$$

The lifecycle node executes one serialized probe cycle: calculate an action, move to its high-clearance XY approach pose, probe along the action direction `N`, capture the first guarded contact, retract along `-N` by `safety.retract_distance`, update the GP/factor belief, and calculate the next action. The first action uses map-frame `N = (0, 0, -1)`; later directions are normalized and constrained to `action.probe_cone_angle_deg` around map-frame `-Z`. The tool `+Z` axis is aligned with `-N`, so the pad normal is opposite the motion direction. The remaining orientation variable is axial spin around `N`, using `action.yaw_bounds`; roll and pitch are derived from `N` and are not independently optimized. The selected approach pose is published at `~/next_omega_target` as a full Cartesian pose. For every accepted contact, the node resolves `omega_contact_tip` into `tactile_map`, so GP samples use the calibrated physical tip XYZ and measured RPY rather than the Franka end-effector frame. The math is split into `scripts/action_model.py` for action and pose geometry, `scripts/gp_models.py` for GP/forward prediction, `scripts/belief_model.py` for factor updates, and `scripts/exploration_policy.py` for optimization. Together they represent the continuous action with position, probe direction, axial spin, velocity, normal force, duration, and interaction mode, then optimize $\lambda_1 I(\mathbf{s},\eta;\mathbf{z}|\mathbf{u},\mathcal{B}) + \lambda_2 a(\mathbf{x},\mathbf{u}) - \lambda_3 C(\mathbf{u})$ inside configured action bounds. The selected action uses the GP geometry estimate for `Z` plus `approach_height`; the probe endpoint is bounded by `action.max_probe_distance`. `scripts/omega_math.py` remains as a compatibility facade for older imports.

`scripts/moveit_executor.py` provides the collision-checked MoveIt `/move_action` client. `scripts/mock_square_test.py` uses that client to execute the opt-in `motion_test.*` XY square through the mock Panda controller; those square points are test fixtures only and are separate from active action generation. This mock test uses the `world` frame and must not be treated as authorization to command physical hardware. Real-arm execution requires its own safety-reviewed state machine, workspace limits, retract behavior, and controller-side safety configuration.

## Tunable Framework Modules

All first-stage entries are parameters in `config/omega_explorer.yaml`:

| Framework module | Parameters |
| --- | --- |
| Observation space | Omega full-taxel topic and `sensor.measurement_stddev` |
| Action space | `action.*` continuous bounds, `approach_height`, and generated pose/motion action |
| Material and geometry GP | `geometry.*`, `material.*` |
| Interaction process | Static/elastic prior, fused contact metric weights, and thresholds |
| Sensor model | `LearnedTactileForwardModel`, dense `239 x 3` Omega field, calibrated taxel positions/normals, `forward_model.ridge`, and Franka external-force estimate |
| Affordance/reward | action-conditioned latent regimes, `objective.lambda_*`, and reward model |
| Safety/cost | `safety.*`; the execution adapter must reject poses outside RPY/workspace limits and retract along map +Z |
| Elite extension | Add an Elite sensor strategy that supplies samples and candidate actions through the same map interface |

## Translation-Only `tactile_map` Calibration

Keep the table level relative to `panda_link0` and hold the Omega orientation fixed. Use a flat calibration plate with known points. At each point, make a low-force contact and record the contact position from `FrankaState.o_t_ee` after applying the known pad-to-EE offset. The translation is:

$$
{}^{\mathrm{panda\_link0}}\mathbf{t}_{\mathrm{tactile\_map}} = \frac{1}{N}\sum_i\left(\mathbf{p}^{\mathrm{panda\_link0}}_i - \mathbf{p}^{\mathrm{tactile\_map}}_i\right)
$$

Set that three-element result as `map_translation_xyz` in both the explorer configuration and static TF publisher. The `omega_mapping.launch.py` launch arguments also expose `map_roll`, `map_pitch`, and `map_yaw` for the tactile-map orientation. Verify the full six-DoF calibration residual at multiple plate points before enabling real-arm descent.