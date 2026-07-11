"""
Step 3: Build a 4-region foot-contact variant of 2D_gait.osim.

Starting from the shipped 2D_gait.osim (which has heel + front contact spheres
on each foot), produce 2D_gait_4regions.osim with FOUR regions per foot:
   heel  ->  kept as-is
   front -> replaced with midfoot + forefoot + toe (three new spheres)

Why XML manipulation instead of pure OpenSim Python? Because the forces and
spheres in 2D_gait.osim live inside the model's top-level <components> block
and <ContactGeometrySet>, and OpenSim's Python API does not expose a clean
way to REMOVE existing entries from those containers. Editing the XML is the
simplest reliable route. The resulting .osim file is just as valid as one
emitted by model.printToXML().
"""
import xml.etree.ElementTree as ET
import shutil

SRC = "E:/OpenSim/4.5/Code/Matlab/Moco/example2DWalking/2D_gait.osim"
DST = "E:/OpenSim/models/2D_gait_4regions.osim"

# Shared sphere physics (matches values used by the shipped model).
PHYS = dict(
    stiffness="3067776",
    dissipation="2",
    static_friction="0.8",
    dynamic_friction="0.8",
    viscous_friction="0.5",
    transition_velocity="0.2",
    derivative_smoothing="1e-05",
    hertz_smoothing="300",
    hunt_crossley_smoothing="50",
)

# New regions on the calcaneus, in body-local coordinates.
# x is forward, y is up, z is medial (negative for right foot per OpenSim convention).
# These values are tuned roughly to a 26-cm adult foot; refine for your subject.
NEW_REGIONS = {
    "midfoot":  dict(loc="0.090 -0.005 0.0", radius="0.025"),
    "forefoot": dict(loc="0.150 -0.012 0.0", radius="0.020"),
    "toe":      dict(loc="0.205 -0.005 0.0", radius="0.018"),
}

# ---------------------------------------------------------------------------
# Parse + transform
# ---------------------------------------------------------------------------
shutil.copyfile(SRC, DST)                       # start from a copy
tree = ET.parse(DST)
root = tree.getroot()

model = root.find("Model")
components = model.find("components")
geom_set = model.find("ContactGeometrySet/objects")

# 1. Remove the two existing "front" entries (forces + spheres).
for name in ("contactFront_r", "contactFront_l"):
    el = components.find(f"SmoothSphereHalfSpaceForce[@name='{name}']")
    if el is not None:
        components.remove(el)
for name in ("front_r", "front_l"):
    el = geom_set.find(f"ContactSphere[@name='{name}']")
    if el is not None:
        geom_set.remove(el)


def _make_force(name, sphere_name):
    """Create a SmoothSphereHalfSpaceForce XML element."""
    f = ET.SubElement(components, "SmoothSphereHalfSpaceForce", name=name)
    ET.SubElement(f, "socket_sphere").text = f"/contactgeometryset/{sphere_name}"
    ET.SubElement(f, "socket_half_space").text = "/contactgeometryset/floor"
    for k, v in PHYS.items():
        ET.SubElement(f, k).text = v
    return f


def _make_sphere(name, body_path, loc, radius):
    """Create a ContactSphere XML element."""
    s = ET.SubElement(geom_set, "ContactSphere", name=name)
    ET.SubElement(s, "socket_frame").text = body_path
    ET.SubElement(s, "radius").text = radius
    ET.SubElement(s, "location").text = loc


# 2. Add midfoot / forefoot / toe spheres for both feet.
for region, params in NEW_REGIONS.items():
    for side in ("r", "l"):
        sphere_name = f"{region}_{side}"                       # e.g. midfoot_r
        force_name = f"contact{region.capitalize()}_{side}"    # contactMidfoot_r
        body_path = f"/bodyset/calcn_{side}"
        # Mirror z for the left foot.
        loc_parts = params["loc"].split()
        if side == "l":
            loc_parts[2] = f"{-float(loc_parts[2]):g}"
        loc = " ".join(loc_parts)
        _make_sphere(sphere_name, body_path, loc, params["radius"])
        _make_force(force_name, sphere_name)

# Indent output so the file is human-readable when opened in a text editor.
ET.indent(tree, space="    ")
tree.write(DST, encoding="UTF-8", xml_declaration=True)

print(f"Wrote {DST}")
print("Final foot contact regions per side:  heel (kept)  +  midfoot  +  forefoot  +  toe")
print("\nVerify the file by:")
print("  1. Open it in the OpenSim GUI -> right-click model -> Display ->")
print("     Contact Geometry -> Show. You should see 8 spheres on the feet.")
print("  2. Re-run script 02 against this model (update its MODEL constant)")
print("     to log per-region forces.")
