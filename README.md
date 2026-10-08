# Phone to Panda

**From personally recorded phone video to simulated robot manipulation.**

Phone to Panda explores how a human pushing demonstration can be transferred to a Franka Panda arm in MuJoCo. A pink marker attached to a finger is tracked through the recording, and its movement drives the robot as it pushes a block towards a target.

The project develops progressively, starting with a reproducible demonstration-replay baseline and controlled evaluation, with feedback control and imitation learning planned for later versions.

## Current implementation

The first version implements **open-loop retargeted demonstration replay**:

- Track a coloured finger marker using HSV colour filtering.
- Smooth the trajectory, select the pushing segment and normalise horizontal movement.
- Map the recorded progress to a predefined Cartesian hand path.
- Calculate joint commands using numerical inverse kinematics with fixed hand orientation.
- Replay the movement with robot gravity compensation and simulated contact physics.
- Evaluate different block starting positions and compare recorded timing against scripted timing.

The current controller follows the demonstration without adjusting its actions to the observed block position. Policy training, VLA models, world models, reinforcement learning and LIBERO integration are future extensions.

## How it works

| Stage | Output |
|---|---|
| Phone recording | Video of a finger pushing a box |
| Marker tracking | Horizontal and vertical marker coordinates in pixels |
| Trajectory processing | Smoothed, trimmed movement progress from 0 to 1 |
| Retargeting | Desired progress along a fixed robot-hand path |
| Pose inverse kinematics | Joint configurations matching hand position and orientation |
| Simulation | Panda finger contact and block movement |
| Evaluation | Target distance, contact, support, orientation and stability metrics |

In the baseline scene, the block target is 5 cm ahead of its initial centre. The hand travels 5.2 cm, including a nominal 2 mm initial clearance. The gripper remains open and pushes with its existing fingers. Full finger collision bounds determine the initial block placement, and gravity compensation applies to the robot while the block retains normal gravity.

The recording supplies the movement's timing and relative horizontal progress. The scene, hand path and overall travel distance are specified manually.

## Results

The recorded demonstration successfully drove the Panda to contact the block, move it **4.515 cm**, and finish **0.485 cm** from the target in the original starting configuration.

Robustness was evaluated on seven starting positions: the original position, forward offsets of 0.5 cm and 1 cm, and sideways offsets of ±2 cm and ±4 cm. The target remained fixed.

| Controller | Successful episodes | Success rate | Mean target distance |
|---|---:|---:|---:|
| Recorded demonstration timing | 3/7 | 42.86% | 2.14 cm |
| Scripted half-cosine timing | 3/7 | 42.86% | 2.12 cm |

Both controllers succeeded on the three centred cases and failed on the four sideways cases. Every episode contacted the block, and the blocks remained supported, upright and nearly stationary. The failures resulted from missing the target position.

These results expose the limitation of the fixed pushing path: changing movement timing did not correct sideways displacement. The rates describe these seven selected simulation cases; they do not establish general manipulation performance or an advantage over scripted control.

## Run in Google Colab

1. Download [the notebook](notebooks/01_phone_to_panda.ipynb) and open it in [Google Colab](https://colab.research.google.com/) using **File → Upload notebook**.
2. Select the **CPU runtime** and execute the cells in order. The notebook embeds the helper module and downloads the robot model automatically.
3. Upload one personally recorded MOV or MP4 when prompted.
4. Inspect the colour mask, tracking preview and selected pushing segment before running the simulation.
5. Run the baseline with rendering disabled, then enable rendering for a selected demonstration video.
6. Run the robustness evaluation and scripted comparison, and download the generated results.

For recording, use a stationary camera, a visible pink finger marker and a contrasting object/background. Push from left to right in the image, with a short pause before and after the movement. MOV support depends on the video's codec.

Rendering defaults to **off** for faster evaluation. Optional videos use **320 × 240 resolution at 10 FPS**, without changing the physics timestep.

### Dependencies and reproducibility

The notebook uses **MuJoCo 3.15.0** and pins the robot model to MuJoCo Menagerie commit:

```text
0059d4335f8156206f63a35662313385f7ad6d74
```

The first setup requires internet access. CPU rendering in Colab uses OSMesa. Dependency specifications are provided in `requirements.txt`, with the exact packaged-code validation environment recorded in `requirements-tested.txt`.

Exported run metadata records the model commit, MuJoCo version, video hash, tracking configuration and selected demonstration duration.

## Repository structure

| Path | Purpose |
|---|---|
| `notebooks/01_phone_to_panda.ipynb` | Standalone Colab baseline and evaluation workflow |
| `phone_to_panda.py` | Reusable tracking, scene construction, IK, replay and evaluation functions |
| `requirements.txt` | Project dependencies |
| `requirements-tested.txt` | Exact versions used for packaged-code validation |
| `docs/experiment_log.md` | Development milestones and earlier experimental observations |
| `docs/evaluation_protocol.md` | Test cases, metrics and comparison procedure |
| `docs/validation.md` | Packaged-code validation, including explicitly identified synthetic checks |
| `data/raw/` | Local demonstration recordings |
| `results/` | Selected exported metrics, trajectories, metadata and simulation videos |

Raw recordings and generated results are ignored by Git by default. Selected results can be committed explicitly when documenting an experiment.

## Evaluation criteria

**Position success** requires the final block centre to be within **1 cm** of the target in the tabletop plane.

The stricter **success** metric also requires:

- Detected robot–block contact.
- The block remaining supported on the tabletop.
- Block tilt below 15°.
- Final linear speed below 0.02 m/s.

Initial robot overlaps and unsupported block placements are reported as invalid starts and excluded from completed-episode success rates. The tabletop support test is an approximation for near-upright blocks.

The recorded and scripted controllers use the same robot path, scene, selected duration, settling period and final hold. Only their progress timing differs.

## Limitations

- Tracking selects the largest colour-matching region. Visual inspection is needed to confirm marker identity.
- Frame timing assumes a constant frame rate based on the video's reported FPS.
- Camera calibration and perspective correction are not implemented; marker positions remain image-space measurements.
- Horizontal motion is manually scaled to a predefined robot path. The demonstration does not supply calibrated 3D motion or robot action labels.
- The controller does not adapt to changes in block position during execution.
- The IK solver respects joint limits but does not perform collision-aware motion planning.
- Gravity compensation is an ideal simulation aid; deployment on a physical robot would require additional control and validation.

## Roadmap

- [x] Track a personally recorded manipulation demonstration.
- [x] Retarget movement to a simulated Panda arm.
- [x] Push a block using the robot's existing fingers.
- [x] Evaluate starting-position changes and compare against scripted timing.
- [ ] Add feedback control using observed block state.
- [ ] Collect multiple demonstrations and design an imitation-learning dataset.
- [ ] Train and evaluate a small policy.
- [ ] Explore transfer to LIBERO and compatible VLA approaches.

## Acknowledgements

Robot model: [Google DeepMind's MuJoCo Menagerie — Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda).

Physics engine: [MuJoCo](https://github.com/google-deepmind/mujoco).

Robot assets are downloaded from the upstream project and are not redistributed here. The Panda model is provided under Apache-2.0; consult its upstream licence and retain the relevant notices when redistributing assets.

The repository's original code has no project licence specified yet.
