# Robustness evaluation

## What changes

Block initial xy position only. Cases: original; x +0.5 cm; x +1 cm; y +2 cm; y -2 cm; y +4 cm; y -4 cm. Dimensions, mass, friction, gravity, target and robot path remain fixed. The target is not shifted with the block. Starting overlaps and blocks not fully supported on the tabletop are marked invalid instead of counting them as completed failures.

## What is compared

- Demo controller: progress from the recorded and inspected marker trajectory.
- Scripted controller: half-cosine progress over the same selected duration.

Both use the same precomputed pose-IK path, 0.5 s initial settling and 1 s final hold. No images are rendered during the evaluation suite. Each case is deterministic and repeated runs with identical inputs do not establish statistical confidence or add new independent trials.

## Metrics

`distance_cm`: final xy centre distance to fixed target. `block_travel_cm`: final minus initial x. `robot_contacted_block`: detected nonpositive contact distance with a robot geometry. `position_success`: distance < 1 cm. `on_table`: centre height within 5 mm of expected resting height and unrotated block footprint fully within tabletop bounds. `upright`: local z axis within 15 degrees of world z. `final_speed_m_s`: linear speed of the free-joint block. `success`: all physical checks plus final speed < 0.02 m/s. The support check is an approximation for near-upright blocks, not a general geometric support estimator.

## Reporting

Report the number of completed episodes, invalid starts, strict successes, position successes, distances and failure videos for selected cases. The success percentage applies to the selected suite only. Testing varied demonstrations, noise, lighting and geometry is future work. Stronger performance claims require broader independently selected tests.

Do not treat successful replay as a learned policy or evidence of generalisation. This evaluation is designed to expose open-loop limitations.
