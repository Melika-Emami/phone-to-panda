# Phone to Panda

Use a personally recorded phone video to drive a Franka Panda arm pushing a block in MuJoCo. Pink-marker tracking supplies movement progress; pose inverse kinematics converts a short straight hand path into joint commands.

**Current scope:** open-loop retargeted demonstration replay. No policy training, VLA, world model, RL, or LIBERO integration yet. The robot scene and 5 cm block target are manually specified. An angled phone view supplies image-space motion, not calibrated 3D motion.

## Start in Google Colab

1. Download `notebooks/01_phone_to_panda.ipynb` and upload it at https://colab.research.google.com/ (File → Upload notebook).
2. Select CPU runtime and run the cells in order. The notebook embeds the helper module, so uploading the ZIP is unnecessary.
3. Upload your own MOV/MP4 when prompted. Inspect the mask, marker preview and trimming plot before proceeding.
4. Reproduce the baseline with rendering disabled; enable 320 × 240, 10 FPS rendering for a selected video.
5. Download outputs at section 8. Continue to section 9 for no-render robustness tests, then section 10 for comparison with scripted timing.
6. Download the updated results at the end. Retain the raw recording privately or make a deliberate decision about publishing it.

The notebook uses MuJoCo 3.15.0 and pins MuJoCo Menagerie to commit `0059d4335f8156206f63a35662313385f7ad6d74`. Your earlier experimental notebook may use a different version. The first model download requires internet access.

## Pipeline

Phone recording → HSV marker detection → median smoothing → trim pullback → normalised horizontal progress → fixed Cartesian hand path → damped pose IK → joint-position commands → simulated contact → metrics.

The hand travel is 5.2 cm: 5 cm desired push plus a nominal 2 mm gap. Collision bounding boxes are conservative, so actual block travel is measured. Pose IK controls the hand body reference point; full finger geometry is used to place the block. The gripper stays open and pushes with its existing fingers. Robot-only gravity compensation is compiled into the scene.

## Project contents

- `notebooks/01_phone_to_panda.ipynb`: standalone Colab workflow, including evaluation.
- `phone_to_panda.py`: reusable tracking, scene, IK, replay and evaluation helpers.
- `requirements.txt`: dependencies; first-party robot assets are downloaded separately.
- `docs/experiment_log.md`: user-observed milestones and known limitations.
- `docs/evaluation_protocol.md`: test cases, success criteria and fair comparison.
- `docs/validation.md`: packaged-code checks, with synthetic input explicitly identified.
- `data/raw/`: place private demonstration videos here if working locally; ignored by Git.
- `results/`: place downloaded CSVs, metadata and selected MP4s here; ignored by default.

## Evidence and limitations

The original user-run baseline contacted the block, moved it 4.51 cm and finished 0.49 cm from its target, satisfying the initial 1 cm position criterion. This is one reported trial, not an aggregate success rate. The packaged notebook must be rerun with the user's video; the raw video is not included in this project.

Detection coverage does not guarantee marker identity. Tracking picks the largest colour-matching region and assumes reported FPS is constant. No camera calibration or perspective correction is performed. Start/end travel is scaled manually. The scene is deliberately aligned with the demonstrated push. The controller does not observe the block to correct its actions. IK respects joint limits but is not collision-aware. Built-in compensation is an ideal simulation aid rather than a complete real-robot controller.

## Evaluation

Seven fixed starting offsets are compared with a fixed target. Invalid starting collisions or unsupported placement are reported separately. The demo and scripted controllers share path, duration, geometry and metrics; only timing changes. Results produced using synthetic validation trajectories are not presented as performance on personally recorded data.

## Put this on GitHub

1. Extract the project ZIP, then create a repository named `phone-to-panda` (or choose another name and visibility).
2. Upload the extracted project contents, including the notebook under `notebooks/` and helper module at the root. Alternatively use Git commands below after creating an empty repository.
3. Add actual exported results and a selected demo intentionally; raw videos are excluded by default.
4. Update the README with your actual evaluation results before making performance claims.

```bash
git init
git add .
git commit -m "Add phone-video to Panda pushing baseline"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/phone-to-panda.git
git push -u origin main
```

To include selected generated files despite the ignore rule, explicitly add them, for example `git add -f results/baseline_metrics.csv results/robustness_comparison.csv results/demo_push.mp4`. Review their contents before committing. No repository has been created or published automatically.

## Next learning steps

1. Collect several demonstrations and assess tracking/retargeting consistency.
2. Add feedback and vary conditions without aligning the controller to each test.
3. Design observations and action labels for imitation learning.
4. Transfer a suitable task to LIBERO; assess compatibility before choosing a VLA.

## Model attribution

Franka model: Google DeepMind's [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda), Apache-2.0 as specified in that model directory. Robot assets are not redistributed here. MuJoCo: https://github.com/google-deepmind/mujoco. Keep upstream license notices if later redistributing their assets. No license has been chosen for your original project code; choose one before presenting it as licensed open-source software.
