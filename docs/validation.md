# Packaged-code validation

Validated in the execution environment on 8 October 2026 with the dependency versions in `requirements-tested.txt` and the pinned Menagerie commit.

Passed: Python module and notebook cell syntax; notebook schema; robot model loading; no initial robot/block/table overlap; all 41 pose-IK waypoints; synthetic pink-marker video decoding and tracking; trimming and normalisation; no-render dynamics; seven-case evaluation for both controllers; rejection of a deliberately overlapping start. Rendering was checked using EGL in the execution environment. The notebook selects OSMesa for CPU Colab; that backend was already used successfully in the user's original notebook, but the complete packaged notebook has not been executed in the user's Colab session.

A synthetic four-second linear progress trajectory contacted the block, moved it 4.257 cm and finished 0.743 cm from the target. It passed the physical checks. This is a smoke check, **not a result on the user's personally collected video**. Synthetic numerical checks exposed the expected open-loop sideways failures. These checks are not published as dataset performance.

The original video remains in the user's Colab session and was not available for this build. Re-upload it to the consolidated notebook, visually inspect tracking and trimming, then run the actual evaluation. Do not claim the packaged version has reproduced the original recorded-data metrics until that is done.
