"""Phone-marker trajectory -> pose IK -> Panda pushing in MuJoCo.

Set MUJOCO_GL before importing this module when rendering in Colab.
"""
from pathlib import Path
import json
import subprocess
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

MENAGERIE_COMMIT = "0059d4335f8156206f63a35662313385f7ad6d74"


def download_model(destination):
    destination = Path(destination)
    if not destination.exists():
        subprocess.run(["git", "clone", "--filter=blob:none", "--sparse",
                        "--no-checkout", "https://github.com/google-deepmind/mujoco_menagerie.git",
                        str(destination)], check=True)
    subprocess.run(["git", "-C", str(destination), "sparse-checkout", "set",
                    "franka_emika_panda"], check=True)
    subprocess.run(["git", "-C", str(destination), "checkout", "--detach",
                    MENAGERIE_COMMIT], check=True)
    return destination / "franka_emika_panda"


def detect_marker(frame, lower, upper, min_area=30):
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [c for c in contours if cv2.contourArea(c) > min_area]
    if not candidates:
        return None, mask, None
    contour = max(candidates, key=cv2.contourArea)
    moment = cv2.moments(contour)
    centre = np.array([moment["m10"], moment["m01"]]) / moment["m00"]
    return centre, mask, contour


def track_video(path, lower=(140, 50, 50), upper=(179, 255, 255)):
    lower, upper = np.array(lower, np.uint8), np.array(upper, np.uint8)
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or not np.isfinite(fps) or fps <= 0:
        cap.release()
        raise ValueError("Cannot open video or read valid FPS.")
    records, previews, first_frame = [], [], None
    interval = max(1, round(fps / 2))
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            i = len(records)
            if first_frame is None:
                first_frame = frame.copy()
            centre, _, contour = detect_marker(frame, lower, upper)
            x, y = centre if centre is not None else (np.nan, np.nan)
            records.append({"frame": i, "time_s": i / fps, "x_px": x, "y_px": y})
            if i % interval == 0:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if centre is not None:
                    cv2.drawContours(rgb, [contour], -1, (0, 255, 0), 3)
                    cv2.circle(rgb, tuple(np.round(centre).astype(int)), 8, (0, 255, 0), -1)
                height, width = rgb.shape[:2]
                out_width = min(width, 640)
                previews.append(cv2.resize(rgb, (out_width, round(height * out_width / width))))
    finally:
        cap.release()
    if not records:
        raise ValueError("No frames decoded.")
    return pd.DataFrame(records), fps, first_frame, previews, fps / interval


def prepare_demo(trajectory, fps, end_time=None):
    demo = trajectory.copy()
    if demo[["x_px", "y_px"]].isna().any().any():
        raise ValueError("Missing detections: inspect tracking before making commands.")
    window = max(3, round(fps * 0.15))
    window += window % 2 == 0
    demo["x_smooth"] = demo.x_px.rolling(window, center=True, min_periods=1).median()
    start_x = demo.x_smooth.iloc[:max(1, round(fps * 0.3))].median()
    if end_time is None:
        end_index = demo.x_smooth.idxmax()
    else:
        selected = demo.index[demo.time_s <= end_time]
        if len(selected) < 2:
            raise ValueError("Selected segment is too short.")
        end_index = selected[-1]
    travel = demo.loc[end_index, "x_smooth"] - start_x
    if travel < 10:
        raise ValueError("Less than 10 pixels of left-to-right travel; inspect the plot.")
    selected = demo.loc[:end_index].copy()
    selected["progress"] = ((selected.x_smooth - start_x) / travel).clip(0, 1)
    selected["command_m"] = 0.47 * selected.progress
    return demo, selected


def home_state(model):
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id < 0:
        raise ValueError("Home keyframe is missing.")
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)
    return data


def arm_indices(model):
    ids = np.array([model.joint(f"joint{i}").id for i in range(1, 8)])
    actuators = np.array([model.actuator(f"actuator{i}").id for i in range(1, 8)])
    return model.jnt_qposadr[ids], model.jnt_dofadr[ids], model.jnt_range[ids], actuators


def build_scene(model_directory, travel=0.05, gap=0.002):
    directory = Path(model_directory)
    robot = mujoco.MjModel.from_xml_path(str(directory / "scene.xml"))
    data = home_state(robot)
    finger_ids = [robot.body(name).id for name in ("left_finger", "right_finger")]
    geoms = [i for i in range(robot.ngeom) if robot.geom_bodyid[i] in finger_ids
             and (robot.geom_contype[i] or robot.geom_conaffinity[i])]
    lower, upper = [], []
    for i in geoms:
        rotation = data.geom_xmat[i].reshape(3, 3)
        centre, half = robot.geom_aabb[i].reshape(2, 3)
        world_centre = data.geom_xpos[i] + rotation @ centre
        extent = np.abs(rotation) @ half
        lower.append(world_centre - extent)
        upper.append(world_centre + extent)
    bounds_lower, bounds_upper = np.min(lower, axis=0), np.max(upper, axis=0)
    half_size = np.array([0.02, 0.06, 0.02])
    top = float(bounds_lower[2] - 0.01)
    block_start = np.array([bounds_upper[0] + gap + half_size[0],
                            (bounds_lower[1] + bounds_upper[1]) / 2,
                            top + half_size[2]])
    goal = block_start + np.array([travel, 0, 0])
    table_centre = block_start + np.array([0.04, 0, -half_size[2] - 0.02])
    table_half = np.array([0.13, 0.12, 0.02])
    root = ET.parse(directory / "panda.xml").getroot()
    root.find("compiler").set("meshdir", str((directory / "assets").resolve()))
    for body in root.findall(".//worldbody//body"):
        body.set("gravcomp", "1")
    world = root.find("worldbody")
    text = lambda values: " ".join(str(float(v)) for v in values)
    ET.SubElement(world, "light", pos="0.5 -0.5 1.5", dir="0 0 -1")
    ET.SubElement(world, "geom", name="floor", type="plane", size="1 1 0.1", rgba="0.85 0.85 0.85 1")
    ET.SubElement(world, "geom", name="tabletop", type="box", pos=text(table_centre),
                  size=text(table_half), rgba="0.45 0.35 0.25 1")
    block = ET.SubElement(world, "body", name="push_block", pos=text(block_start))
    ET.SubElement(block, "freejoint", name="block_free")
    ET.SubElement(block, "geom", name="block_geom", type="box", size=text(half_size),
                  mass="0.05", rgba="0.95 0.95 0.95 1")
    ET.SubElement(world, "site", name="push_target", type="box",
                  pos=text([goal[0], goal[1], top + 0.001]),
                  size="0.025 0.065 0.001", rgba="0.2 0.8 0.3 0.6")
    for key in root.findall("./keyframe/key"):
        key.set("qpos", key.get("qpos") + " " + text([*block_start, 1, 0, 0, 0]))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    info = {"block_start": block_start, "goal": goal, "table_top": top,
            "table_centre": table_centre, "table_half": table_half,
            "block_half": half_size, "hand_travel": travel + gap,
            "menagerie_commit": MENAGERIE_COMMIT}
    return model, info


def robot_scene_contacts(model, data):
    block = model.geom("block_geom").id
    table = model.geom("tabletop").id
    block_body = model.body("push_block").id
    records = []
    for contact in data.contact[:data.ncon]:
        g1, g2 = int(contact.geom1), int(contact.geom2)
        for scene, other in ((g1, g2), (g2, g1)):
            if scene in (block, table) and model.geom_bodyid[other] not in (0, block_body):
                records.append({"scene": model.geom(scene).name,
                                "robot_body": model.body(model.geom_bodyid[other]).name,
                                "distance_mm": float(contact.dist * 1000)})
    return records


def solve_pose_ik(model, position, rotation, initial, max_iterations=200):
    qids, dids, limits, _ = arm_indices(model)
    data = mujoco.MjData(model)
    data.qpos[:] = initial
    hand = model.body("hand").id
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for _ in range(max_iterations):
        mujoco.mj_forward(model, data)
        ep = position - data.body("hand").xpos
        er = Rotation.from_matrix(rotation @ data.body("hand").xmat.reshape(3, 3).T).as_rotvec()
        if np.linalg.norm(ep) < 0.001 and np.linalg.norm(er) < np.deg2rad(0.5):
            break
        mujoco.mj_jacBody(model, data, jp, jr, hand)
        J = np.vstack([jp[:, dids], 0.2 * jr[:, dids]])
        error = np.r_[ep, 0.2 * er]
        dq = J.T @ np.linalg.solve(J @ J.T + 0.01**2 * np.eye(6), error)
        dq *= min(1.0, 0.05 / max(np.max(np.abs(dq)), 1e-12))
        data.qpos[qids] = np.clip(data.qpos[qids] + dq, limits[:, 0], limits[:, 1])
    mujoco.mj_forward(model, data)
    pe = np.linalg.norm(position - data.body("hand").xpos)
    re = np.linalg.norm(Rotation.from_matrix(
        rotation @ data.body("hand").xmat.reshape(3, 3).T).as_rotvec())
    return data.qpos.copy(), float(pe), float(re)


def plan_push(model, info, count=41):
    data = home_state(model)
    qids, _, _, _ = arm_indices(model)
    start = data.body("hand").xpos.copy()
    rotation = data.body("hand").xmat.reshape(3, 3).copy()
    fractions = np.linspace(0, 1, count)
    commands, errors, guess = [], [], data.qpos.copy()
    for fraction in fractions:
        target = start + np.array([info["hand_travel"] * fraction, 0, 0])
        solution, pe, re = solve_pose_ik(model, target, rotation, guess)
        if pe >= 0.001 or re >= np.deg2rad(0.5):
            raise ValueError(f"IK failed at progress={fraction:.3f}: {pe*1000:.2f} mm, {np.rad2deg(re):.2f} deg")
        commands.append(solution[qids])
        errors.append([pe, re])
        guess = solution
    return {"fractions": fractions, "commands": np.array(commands),
            "errors": np.array(errors), "hand_start": start, "hand_rotation": rotation}


def run_episode(model, info, plan, demo, offset=(0, 0), render=False, fps=10,
                width=320, height=240, controller="demo"):
    if controller not in ("demo", "scripted"):
        raise ValueError("controller must be demo or scripted")
    data = home_state(model)
    block_joint = model.joint("block_free").id
    qadr, vadr = model.jnt_qposadr[block_joint], model.jnt_dofadr[block_joint]
    data.qpos[qadr:qadr+2] += np.array(offset)
    mujoco.mj_forward(model, data)
    initial = data.body("push_block").xpos.copy()
    overlaps = [r for r in robot_scene_contacts(model, data) if r["distance_mm"] < -0.1]
    supported = np.all(np.abs(initial[:2] - info["table_centre"][:2])
                       + info["block_half"][:2] <= info["table_half"][:2] + 1e-9)
    if overlaps or not supported:
        return {"status": "invalid_start", "reason": "robot overlap" if overlaps else "block outside tabletop",
                "controller": controller, "success": False}, [], pd.DataFrame()
    _, _, _, aids = arm_indices(model)
    home_ctrl = data.ctrl.copy()
    times, progress = demo.time_s.to_numpy(), demo.progress.to_numpy()
    if len(times) < 2 or times[-1] <= 0:
        raise ValueError("Demonstration has no usable duration.")
    settle, hold = 0.5, 1.0
    duration = settle + times[-1] + hold
    renderer = mujoco.Renderer(model, height=height, width=width) if render else None
    camera = mujoco.MjvCamera()
    camera.lookat[:], camera.distance = [0.3, 0, 0.4], 1.8
    camera.azimuth, camera.elevation = 135, -25
    frames, logs, next_sample, touched = [], [], 0.0, False
    block_geom = model.geom("block_geom").id
    try:
        for _ in range(int(np.ceil(duration / model.opt.timestep))):
            t = data.time - settle
            if t < 0:
                blend = 0.0
            elif controller == "demo":
                blend = np.interp(t, times, progress)
            else:
                blend = (1 - np.cos(np.pi * np.clip(t / times[-1], 0, 1))) / 2
            data.ctrl[:] = home_ctrl
            data.ctrl[aids] = [np.interp(blend, plan["fractions"], plan["commands"][:, j]) for j in range(7)]
            mujoco.mj_step(model, data)
            for c in data.contact[:data.ncon]:
                g1, g2 = int(c.geom1), int(c.geom2)
                if block_geom in (g1, g2):
                    other = g2 if g1 == block_geom else g1
                    if model.geom_bodyid[other] not in (0, model.body("push_block").id) and c.dist <= 0:
                        touched = True
            if data.time >= next_sample:
                mujoco.mj_forward(model, data)
                xyz = data.body("push_block").xpos.copy()
                logs.append({"time_s": data.time, "command_progress": float(blend),
                             "block_x_m": xyz[0], "block_y_m": xyz[1], "block_z_m": xyz[2]})
                if renderer:
                    renderer.update_scene(data, camera=camera)
                    frames.append(renderer.render().copy())
                next_sample += 1 / fps
    finally:
        if renderer:
            renderer.close()
    mujoco.mj_forward(model, data)
    final = data.body("push_block").xpos.copy()
    distance = float(np.linalg.norm(final[:2] - info["goal"][:2]))
    speed = float(np.linalg.norm(data.qvel[vadr:vadr+3]))
    upright = bool(data.body("push_block").xmat.reshape(3, 3)[2, 2] > np.cos(np.deg2rad(15)))
    on_table = bool(abs(final[2] - info["block_start"][2]) < 0.005 and
                    np.all(np.abs(final[:2] - info["table_centre"][:2]) + info["block_half"][:2]
                           <= info["table_half"][:2] + 1e-6))
    result = {"status": "completed", "controller": controller,
              "robot_contacted_block": bool(touched), "block_travel_cm": float((final[0] - initial[0]) * 100),
              "distance_cm": distance * 100, "final_x_cm": final[0] * 100,
              "final_y_cm": final[1] * 100, "final_speed_m_s": speed,
              "on_table": on_table, "upright": upright, "position_success": bool(distance < 0.01),
              "success": bool(touched and distance < 0.01 and on_table and upright and speed < 0.02)}
    return result, frames, pd.DataFrame(logs)


ROBUSTNESS_CASES = [("original", 0, 0), ("x +0.5 cm", 0.005, 0),
                    ("x +1 cm", 0.01, 0), ("y +2 cm", 0, 0.02),
                    ("y -2 cm", 0, -0.02), ("y +4 cm", 0, 0.04),
                    ("y -4 cm", 0, -0.04)]


def evaluate(model, info, plan, demo, controller="demo"):
    rows = []
    for name, dx, dy in ROBUSTNESS_CASES:
        result, _, _ = run_episode(model, info, plan, demo, offset=(dx, dy), controller=controller)
        rows.append({"test": name, "offset_x_cm": dx * 100, "offset_y_cm": dy * 100, **result})
        print(f"{controller}: {name}: {result['status']}, success={result['success']}")
    return pd.DataFrame(rows)


def save_run_metadata(path, info, demo, extra=None):
    metadata = {"mujoco_version": mujoco.__version__, "menagerie_commit": MENAGERIE_COMMIT,
                "selected_demo_duration_s": float(demo.time_s.iloc[-1]),
                "block_start_m": info["block_start"].tolist(), "goal_m": info["goal"].tolist(),
                "hand_travel_m": info["hand_travel"], "controller_type": "open-loop retargeted replay",
                **(extra or {})}
    Path(path).write_text(json.dumps(metadata, indent=2) + "\n")
