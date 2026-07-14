from pathlib import Path
from shutil import which

try:
    from graphviz import Digraph
    from graphviz.backend.execute import ExecutableNotFound
except ImportError as exc:
    raise SystemExit(
        "Missing Python package 'graphviz'. Install it with:\n"
        "  python -m pip install graphviz"
    ) from exc

# -----------------------------
# Basic style settings
# -----------------------------
RED = "#e11b0c"
LIGHT_GRAY = "#f2f2f2"
BLACK = "#111111"
WHITE = "#ffffff"

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_STEM = SCRIPT_DIR / "simulation_architecture_overview"
DOT_PATH = OUTPUT_STEM.with_suffix(".dot")

def build_diagram():
    dot = Digraph("Simulation_Software_Architecture", format="png")

    dot.attr(
        rankdir="LR",
        bgcolor=LIGHT_GRAY,
        splines="ortho",
        nodesep="0.8",
        ranksep="1.0",
        fontname="Arial",
    )

    dot.attr(
        "node",
        shape="rect",
        style="rounded,filled",
        fillcolor=RED,
        color=BLACK,
        fontcolor=WHITE,
        fontname="Arial",
        fontsize="14",
        penwidth="1.5",
        margin="0.15",
    )

    dot.attr(
        "edge",
        color=BLACK,
        fontname="Arial",
        fontsize="11",
        arrowsize="0.8",
    )

    dot.node(
        "title",
        "Simulation Software Architecture Overview\nROS 2 launch structure for mecanum ASR simulation",
        shape="plaintext",
        fontcolor=BLACK,
        fontsize="24",
    )

    dot.node("xacro", "mecanum_bot.urdf.xacro")
    dot.node("rsp", "robot_state_publisher")
    dot.node("spawn", "spawn_entity.py")

    with dot.subgraph(name="cluster_gazebo") as gz:
        gz.attr(
            label="Gazebo simulation\nnav_test_room.world",
            style="rounded,filled",
            color=BLACK,
            fillcolor=RED,
            fontcolor=WHITE,
            fontsize="18",
            fontname="Arial",
            penwidth="1.8",
        )

        gz.node("mecanum", "mecanum_bot", fontsize="12")
        gz.node("charging", "ChargingZone", fontsize="12")
        gz.node("drop", "DropZone", fontsize="12")
        gz.node("orange_drop", "OrangeDropZone", fontsize="12")
        gz.node("trolley", "Trolley", fontsize="12")

    dot.node("front_aruco", "front_aruco_node")
    dot.node("rear_aruco", "rear_aruco_node")
    dot.node("left_aruco", "left_aruco_node")
    dot.node("right_aruco", "right_aruco_node")

    dot.node("slam", "SLAM Toolbox")
    dot.node("nav2", "Nav2")

    dot.node("mission", "nav2_trolley_\nmission_controller.py")
    dot.node("holonomic", "holonomic_goal_\ncontroller.py")

    dot.node(
        "optional",
        "Optional, not launched:\n\n- YOLO object detection\n- trolley_ready_docking_controller",
        shape="rect",
        style="rounded,dashed",
        fillcolor=LIGHT_GRAY,
        fontcolor=BLACK,
        color=RED,
        fontsize="12",
    )

    dot.edge("title", "xacro", style="invis")

    dot.edge("xacro", "rsp")
    dot.edge("rsp", "spawn", label="robot_description")
    dot.edge("rsp", "nav2", label="/tf\n/tf_static")
    dot.edge("spawn", "mecanum", label="spawn robot + models")

    dot.edge("mecanum", "front_aruco", label="/camera_front/image_raw")
    dot.edge("mecanum", "rear_aruco", label="/camera_rear/image_raw")
    dot.edge("mecanum", "left_aruco", label="/camera_left/image_raw")
    dot.edge("mecanum", "right_aruco", label="/camera_right/image_raw")

    dot.edge("front_aruco", "mission", label="/front/aruco_markers")
    dot.edge("rear_aruco", "mission", label="/rear/aruco_markers")
    dot.edge("left_aruco", "mission", label="/left/aruco_markers")
    dot.edge("right_aruco", "mission", label="/right/aruco_markers")

    dot.edge("mecanum", "slam", label="/scan")
    dot.edge("mecanum", "nav2", label="/odom")
    dot.edge("slam", "nav2", label="/map")

    dot.edge("nav2", "mission", label="navigation state\npose, plan, costmap")

    dot.edge("mission", "holonomic", label="/holonomic_goal")
    dot.edge("mission", "holonomic", label="/holonomic_cancel")
    dot.edge("mission", "holonomic", label="/holonomic_lidar_ignore_radius")

    dot.edge("holonomic", "mecanum", label="/cmd_vel")

    dot.edge("optional", "mission", style="dashed", label="optional perception/control")

    return dot


def main():
    dot = build_diagram()

    if which("dot") is None:
        DOT_PATH.write_text(dot.source, encoding="utf-8")
        raise SystemExit(
            "Graphviz executable 'dot' was not found on PATH.\n"
            "Only the DOT source can be saved until Graphviz is installed.\n"
            "Install Graphviz and make sure its bin directory is on PATH, then run this script again.\n"
            "Windows installer: https://graphviz.org/download/\n"
            f"DOT source saved to: {DOT_PATH}"
        )

    try:
        DOT_PATH.write_text(dot.source, encoding="utf-8")
        output_paths = [
            dot.render(str(OUTPUT_STEM), format=output_format, cleanup=True)
            for output_format in ("png", "eps")
        ]
    except ExecutableNotFound as exc:
        DOT_PATH.write_text(dot.source, encoding="utf-8")
        raise SystemExit(
            "Graphviz executable 'dot' was not found on PATH.\n"
            "Only the DOT source can be saved until Graphviz is installed.\n"
            "Install Graphviz and make sure its bin directory is on PATH, then run this script again.\n"
            "Windows installer: https://graphviz.org/download/\n"
            f"DOT source saved to: {DOT_PATH}"
        ) from exc

    print("Diagram files saved:")
    print(f"  DOT: {DOT_PATH}")
    for output_path in output_paths:
        print(f"  {output_path}")


if __name__ == "__main__":
    main()
