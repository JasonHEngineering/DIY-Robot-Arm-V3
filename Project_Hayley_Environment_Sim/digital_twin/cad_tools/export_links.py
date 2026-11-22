"""
FreeCAD (headless) exporter: splits Robot_Assembly.step into per-link STL meshes
expressed in each URDF link frame.

Run with FreeCAD's command-line interpreter, from anywhere:
    "C:/Users/<you>/AppData/Local/Programs/FreeCAD 1.1/bin/freecadcmd.exe" -c "exec(open(r'<path>/export_links.py').read())"

Steps
 1. Import the assembly STEP (it is modelled at the kinematic-zero pose, Z up,
    but rotated ~14.4 deg about Z and offset in XY).
 2. Find the base frame from the CAD itself: J1 axis = centre of the J1 harmonic
    reducer ("Part 044"), forward (+X) = J4 reducer axis ("Part 041"), Z=0 at the
    top of the base plate ("Part 009"). This lines up with the linkage table in
    hayley_kinematics.py to <0.1 mm.
 3. Classify every solid into base_link / link1..link6 with geometric rules
    (overridable through link_overrides.json: {"Part 034": "link4", ...}).
 4. Group each link's solids by category (structure / drive / motor / pcb /
    hardware), mesh, move into the link frame (origin = joint origin at home)
    and write binary STL (mm units, URDF applies scale 0.001).
"""
import os, sys, json, math

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else None
if HERE is None:
    # exec(open(...).read()) path: find ourselves via argv or a fixed relative guess
    HERE = os.environ.get("HAYLEY_TOOLS_DIR", os.getcwd())
sys.path.insert(0, HERE)

import FreeCAD, Import, Part, MeshPart
from FreeCAD import Vector, Matrix
import hayley_kinematics as hk

ROOT = os.path.dirname(HERE)
STEP = os.path.join(ROOT, "cad_source", "Robot_Assembly.step")
OUT_DIR = os.path.join(ROOT, "description", "meshes")
MAP_OUT = os.path.join(ROOT, "description", "link_map.json")
OVERRIDES = os.path.join(HERE, "link_overrides.json")
LIN_DEFL = 0.15   # mm (structure / drive / motor)
LIN_DEFL_BY_CAT = {"hardware": 0.5, "pcb": 0.4}  # coarser for screws and PCB bits
ANG_DEFL = 0.4    # rad
MIN_VOL = 2.0     # mm^3 - drop tiny SMD bits
os.makedirs(OUT_DIR, exist_ok=True)

LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]
joints, _ = hk.home_joints()
LINK_ORIGIN = {"base_link": [0.0, 0.0, 0.0]}
for i, j in enumerate(joints, 1):
    LINK_ORIGIN["link%d" % i] = j["origin"]


def circle_of(obj, radius):
    for e in obj.Shape.Edges:
        try:
            c = e.Curve
        except Exception:
            continue
        if isinstance(c, Part.Circle) and abs(c.Radius - radius) < 0.05:
            return c
    raise RuntimeError("circle r=%s not found on %s" % (radius, obj.Label))


doc = FreeCAD.newDocument("Hayley")
Import.insert(STEP, doc.Name)
objs = [o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Shape.Solids]
by_label = {o.Label: o for o in objs}

# ---- base frame from CAD ----------------------------------------------------
ax = circle_of(by_label["Part 041"], 20.5).Axis
X = Vector(ax.x, ax.y, 0); X.normalize()
if X.y < 0:
    X = -X
Z = Vector(0, 0, 1); Y = Z.cross(X)
c1 = circle_of(by_label["Part 044"], 20.5).Center
O = Vector(c1.x, c1.y, by_label["Part 009"].Shape.BoundBox.ZMax)
M = Matrix(X.x, X.y, X.z, 0, Y.x, Y.y, Y.z, 0, Z.x, Z.y, Z.z, 0, 0, 0, 0, 1)
t = M.multiply(Vector(-O.x, -O.y, -O.z)); M.A14, M.A24, M.A34 = t.x, t.y, t.z
print("base frame: yaw=%.3f deg, origin=%s" % (math.degrees(math.atan2(X.y, X.x)) - 90, O))


# ---- classification --------------------------------------------------------
def classify(c, bb):
    ay = abs(c.y)
    if bb.ZMin < 50:
        return "base_link"
    if c.z < 100:
        return "link1" if math.hypot(c.x, c.y) < 24 else "base_link"
    if c.z < 170:
        return "link2" if ay > 62 else "link1"
    if c.z < 240 and c.x < 22:
        return "link2" if ay > 62 else "link3"
    if c.x < 30:
        return "link3"
    if c.x >= 186:
        return "link6"
    if c.x >= 125:
        return "link5"
    if 24 <= ay < 46 and c.x > 65:
        return "link5"
    return "link4"


def category(label, vol):
    L = label.lower()
    if any(k in L for k in ("17hs", "nema", "pancake", "6020fan")):
        return "motor"
    if any(k in L for k in ("pcb", "raspberry", "encoder", "stepper_with", "header", "torretta")):
        return "pcb"
    if abs(vol - 47862.3) < 1 or any(k in L for k in ("gt2", "shaft", "kfl08", "ring", "bearing")):
        return "drive"
    if vol < 300 or "standoff" in L:
        return "hardware"
    return "structure"


overrides = json.load(open(OVERRIDES)) if os.path.exists(OVERRIDES) else {}
items = []
for o in objs:
    s = o.Shape.copy(); s.transformShape(M)
    vol = s.Volume
    if vol < MIN_VOL:
        continue
    bb = s.BoundBox
    try:
        c = s.CenterOfMass if hasattr(s, "CenterOfMass") else s.Solids[0].CenterOfMass
    except Exception:
        c = bb.Center
    items.append({"o": o, "s": s, "vol": vol, "c": c, "bb": bb})

big = [it for it in items if it["vol"] >= 300]
for it in big:
    it["link"] = classify(it["c"], it["bb"])
# small parts inherit from the smallest big part whose (inflated) bbox holds them
for it in items:
    if "link" in it:
        continue
    best, best_v = None, None
    for b in big:
        bb = b["bb"]
        if (bb.XMin - 2 <= it["c"].x <= bb.XMax + 2 and bb.YMin - 2 <= it["c"].y <= bb.YMax + 2
                and bb.ZMin - 2 <= it["c"].z <= bb.ZMax + 2):
            v = bb.XLength * bb.YLength * bb.ZLength
            if best is None or v < best_v:
                best, best_v = b, v
    if best is None:
        best = min(big, key=lambda b: (b["c"] - it["c"]).Length)
    it["link"] = best["link"]
for it in items:
    it["link"] = overrides.get(it["o"].Label, it["link"])
    it["cat"] = category(it["o"].Label, it["vol"])
# "exclude" in link_overrides.json removes a part from the model (visual + collision)
excluded = [it["o"].Label for it in items if it["link"] == "exclude"]
items = [it for it in items if it["link"] != "exclude"]
if excluded:
    print("excluded:", ", ".join(excluded))
unknown = sorted({it["link"] for it in items} - set(LINKS))
if unknown:
    raise SystemExit("link_overrides.json: unknown link name(s) %s - use base_link, link1..link6 or exclude" % unknown)

# ---- mesh + write ------------------------------------------------------------
groups = {}
for it in items:
    groups.setdefault((it["link"], it["cat"]), []).append(it)

for f in os.listdir(OUT_DIR):
    if f.endswith(".stl"):
        os.remove(os.path.join(OUT_DIR, f))

manifest = {"base_frame": {"cad_origin_mm": list(O), "cad_x_axis": list(X)}, "meshes": [], "parts": []}
for (link, cat), its in sorted(groups.items()):
    comp = Part.makeCompound([it["s"] for it in its])
    org = LINK_ORIGIN[link]
    comp.translate(Vector(-org[0], -org[1], -org[2]))
    mesh = MeshPart.meshFromShape(Shape=comp, LinearDeflection=LIN_DEFL_BY_CAT.get(cat, LIN_DEFL),
                                  AngularDeflection=ANG_DEFL, Relative=False)
    name = "%s_%s.stl" % (link, cat)
    mesh.write(os.path.join(OUT_DIR, name))
    manifest["meshes"].append({"link": link, "category": cat, "file": name,
                               "parts": len(its), "facets": mesh.CountFacets})
    print("wrote", name, len(its), "parts", mesh.CountFacets, "facets")

for it in items:
    manifest["parts"].append({"label": it["o"].Label, "link": it["link"], "category": it["cat"],
                              "volume_mm3": round(it["vol"], 1),
                              "centroid_mm": [round(it["c"].x, 1), round(it["c"].y, 1), round(it["c"].z, 1)]})
with open(MAP_OUT, "w") as f:
    json.dump(manifest, f, indent=1)
print("done:", len(items), "parts ->", len(groups), "meshes")
