"""
Generate worlds/walking_world.sdf: a ground plane, a directional light, an
articulated capsule humanoid, and a pole-mounted camera + gpu_lidar aimed at it.

The humanoid is a fixed-base treadmill walker (pelvis welded to the world frame):
per leg the chain is

    pelvis --hip_flexion(Y)--> hipflex --hip_adduction(X)--> thigh
           --knee(Y)--> shank --ankle(Y)--> foot

matching holotile_sim/skeleton3d.py's FK (forward +x, left +y, up +z; flexion in
the sagittal x-z plane = rotation about Y, adduction in the frontal plane =
rotation about X). Segment lengths are taken straight from skeleton3d so the
Gazebo figure and the perception-FK keypoints are geometrically identical.

Each driven joint carries a JointPositionController listening on
/human/cmd/<joint>; the gait player publishes target angles there. Links are
light and gravity is countered by the controllers, so tracking is tight enough
for the camera/lidar to see a clean walking motion. Ground truth never comes
from Gazebo (it comes from the source trial), so any residual tracking lag does
not affect loop correctness -- only the rendered picture.

Run:  python build_human_sdf.py   (writes worlds/walking_world.sdf)
"""

import os

# Segment lengths (m) -- identical to holotile_sim/skeleton3d.py.
HEIGHT = 1.70
THIGH = 0.245 * HEIGHT      # 0.4165
SHANK = 0.246 * HEIGHT      # 0.4182
FOOT = 0.152 * HEIGHT       # 0.2584
TRUNK = 0.34 * HEIGHT       # 0.578
HIP_HALF = 0.09             # half pelvis width

# Capsule radii (visual + collision so the lidar gets returns).
R_THIGH, R_SHANK, R_FOOT, R_TRUNK, R_PELVIS = 0.060, 0.050, 0.045, 0.085, 0.085
R_UARM, R_FARM = 0.045, 0.038

# Clothed-mannequin colours.
SHIRT = "0.13 0.42 0.20 1"     # green top
PANTS = "0.12 0.15 0.36 1"     # dark-blue trousers
SKIN  = "0.80 0.62 0.48 1"
SHOE  = "0.09 0.09 0.11 1"

# Light links -> the position controllers track the gait reference crisply (this
# is a kinematic visual; dynamics don't matter). The pelvis rides a prismatic
# vertical joint driven per-frame so the stance foot stays on the floor (+ bob).
M_THIGH, M_SHANK, M_FOOT, M_PELVIS = 1.5, 0.8, 0.3, 3.0

# JointPositionController gains. Pelvis I=0 on purpose: integral windup was
# overshooting the moving height reference (feet floating); pure P (12k) lifts the
# light body with a few-mm steady error and no overshoot. (P=25k went unstable ->
# clamped to the joint's lower limit, dropping the body through the floor.)
P_GAIN, I_GAIN, D_GAIN = 600.0, 20.0, 8.0
PZ_P, PZ_I, PZ_D = 12000.0, 0.0, 150.0     # pelvis vertical (holds body weight)

SIDES = ("r", "l")
LEG_JOINTS = ("hip_flexion", "hip_adduction", "knee_angle", "ankle_angle")

# Realistic body-part meshes (extracted from the actor skin, in link frames).
MESH_BASE = "file://" + os.path.dirname(os.path.abspath(__file__)) + "/meshes"
def _mesh_uri(name):
    return f"{MESH_BASE}/{name}.obj"


def _inertial(mass, r, length):
    """Solid-capsule-ish diagonal inertia (cylinder approximation)."""
    ixx = mass * (3 * r * r + length * length) / 12.0
    izz = 0.5 * mass * r * r
    return (f"<inertial><mass>{mass}</mass><inertia>"
            f"<ixx>{ixx:.5f}</ixx><iyy>{ixx:.5f}</iyy><izz>{izz:.5f}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def _mesh_visual(name, rgba):
    """Realistic body-part mesh visual (already in the link's local frame). Colour
    comes from the mesh's own .mtl; SDF material is omitted (it was ignored and
    rendered white)."""
    return f"""
        <visual name="{name}_v">
          <geometry><mesh><uri>{_mesh_uri(name)}</uri></mesh></geometry>
        </visual>"""


def _capsule_z(name, rel_joint, length, r, rgba, mass):
    """A limb link extending from its origin DOWN by `length` along -z. Visual is
    the extracted body-part mesh; collision stays a cylinder (lidar + physics)."""
    return f"""
      <link name="{name}">
        <pose relative_to="{rel_joint}">0 0 0 0 0 0</pose>
        {_inertial(mass, r, length)}
        {_mesh_visual(name, rgba)}
        <collision name="{name}_c">
          <pose>0 0 {-length/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{length:.4f}</length></cylinder></geometry>
        </collision>
      </link>"""


def _capsule_x(name, rel_joint, length, r, rgba, mass):
    """A limb link extending from its origin FORWARD by `length` along +x."""
    return f"""
      <link name="{name}">
        <pose relative_to="{rel_joint}">0 0 0 0 0 0</pose>
        {_inertial(mass, r, length)}
        {_mesh_visual(name, rgba)}
        <collision name="{name}_c">
          <pose>{length/2:.4f} 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>{r}</radius><length>{length:.4f}</length></cylinder></geometry>
        </collision>
      </link>"""


def _massless(name, rel_joint):
    return f"""
      <link name="{name}">
        <pose relative_to="{rel_joint}">0 0 0 0 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>"""


def _joint(name, jtype, parent, child, axis, pose_rel, rel_to):
    axis_xml = ""
    if jtype != "fixed":
        axis_xml = (f"<axis><xyz>{axis}</xyz>"
                    f"<limit><lower>-2.0</lower><upper>2.0</upper><effort>500</effort></limit>"
                    f"<dynamics><damping>0.5</damping></dynamics></axis>")
    return f"""
      <joint name="{name}" type="{jtype}">
        <pose relative_to="{rel_to}">{pose_rel}</pose>
        <parent>{parent}</parent>
        <child>{child}</child>
        {axis_xml}
      </joint>"""


def _controller(joint):
    return f"""
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>/human/cmd/{joint}</topic>
      <p_gain>{P_GAIN}</p_gain><i_gain>{I_GAIN}</i_gain><d_gain>{D_GAIN}</d_gain>
    </plugin>"""


def build_leg(side):
    """One leg's links + joints. y-sign places the hip left(+)/right(-)."""
    ysign = 1.0 if side == "l" else -1.0
    hy = ysign * HIP_HALF
    parts = []
    # hip_flexion: pelvis -> hipflex (tiny), at the hip socket, rotate about Y
    parts.append(_joint(f"hip_flexion_{side}", "revolute", "pelvis",
                        f"hipflex_{side}", "0 1 0", f"0 {hy:.4f} 0 0 0 0", "pelvis"))
    parts.append(_massless(f"hipflex_{side}", f"hip_flexion_{side}"))
    # hip_adduction: hipflex -> thigh, same point, rotate about X
    parts.append(_joint(f"hip_adduction_{side}", "revolute", f"hipflex_{side}",
                        f"thigh_{side}", "1 0 0", "0 0 0 0 0 0", f"hip_flexion_{side}"))
    parts.append(_capsule_z(f"thigh_{side}", f"hip_adduction_{side}", THIGH, R_THIGH, PANTS, M_THIGH))
    # knee: thigh distal -> shank, about Y
    parts.append(_joint(f"knee_angle_{side}", "revolute", f"thigh_{side}",
                        f"shank_{side}", "0 1 0", f"0 0 {-THIGH:.4f} 0 0 0", f"thigh_{side}"))
    parts.append(_capsule_z(f"shank_{side}", f"knee_angle_{side}", SHANK, R_SHANK, PANTS, M_SHANK))
    # ankle: shank distal -> foot, about Y
    parts.append(_joint(f"ankle_angle_{side}", "revolute", f"shank_{side}",
                        f"foot_{side}", "0 1 0", f"0 0 {-SHANK:.4f} 0 0 0", f"shank_{side}"))
    parts.append(_capsule_x(f"foot_{side}", f"ankle_angle_{side}", FOOT, R_FOOT, SHOE, M_FOOT))
    return "".join(parts)


def build_world():
    legs = build_leg("r") + build_leg("l")
    controllers = "".join(_controller(f"{j}_{s}") for s in SIDES for j in LEG_JOINTS)
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="walking">
    <physics name="100hz" type="ignored"><max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <model name="human">
      <pose>0 0 0 0 0 0</pose>
      <link name="pelvis">
        <pose>0 0 0 0 0 0</pose>
        {_inertial(M_PELVIS, R_PELVIS, 2 * HIP_HALF)}
        {_mesh_visual("torso", SHIRT)}
        {_mesh_visual("skin", SKIN)}
        <collision name="torso_c"><pose>0 0 {TRUNK*0.45} 0 0 0</pose>
          <geometry><cylinder><radius>{R_TRUNK}</radius><length>{TRUNK*0.9}</length></cylinder></geometry>
        </collision>
      </link>
      <joint name="anchor" type="prismatic"><parent>world</parent><child>pelvis</child>
        <axis><xyz>0 0 1</xyz>
          <limit><lower>0</lower><upper>1.5</upper><effort>8000</effort></limit>
          <dynamics><damping>5</damping></dynamics></axis>
      </joint>
      {legs}
      <plugin filename="gz-sim-joint-state-publisher-system"
              name="gz::sim::systems::JointStatePublisher"/>
{controllers}
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>anchor</joint_name><topic>/human/cmd/pelvis_ty</topic>
        <p_gain>{PZ_P}</p_gain><i_gain>{PZ_I}</i_gain><d_gain>{PZ_D}</d_gain>
      </plugin>
    </model>

    <model name="sensor_rig">
      <static>true</static>
      <pose>0 -3.2 0.95 0 0 1.5708</pose>
      <link name="link">
        <visual name="v"><geometry><box><size>0.12 0.12 0.12</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
        <sensor name="camera" type="camera">
          <camera><horizontal_fov>1.20</horizontal_fov>
            <image><width>640</width><height>480</height></image>
            <clip><near>0.1</near><far>50</far></clip></camera>
          <always_on>1</always_on><update_rate>30</update_rate><topic>camera</topic>
        </sensor>
        <sensor name="lidar" type="gpu_lidar">
          <lidar><scan>
            <horizontal><samples>320</samples><resolution>1</resolution><min_angle>-0.7</min_angle><max_angle>0.7</max_angle></horizontal>
            <vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.45</min_angle><max_angle>0.45</max_angle></vertical>
          </scan><range><min>0.2</min><max>40</max><resolution>0.005</resolution></range></lidar>
          <always_on>1</always_on><update_rate>20</update_rate><topic>lidar</topic>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "worlds", "walking_world.sdf")
    with open(out, "w") as f:
        f.write(build_world())
    print("wrote", out)
