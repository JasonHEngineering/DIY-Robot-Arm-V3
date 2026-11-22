"""
Single source of truth for the Project Hayley V3 kinematic chain used by the
digital twin tools (URDF generator + FreeCAD mesh exporter).

The linkage table is copied verbatim from `local_linkage_data` in
Project_Hayley_V5_IK_user_interface_V08.py. Each row is
    [axis, theta(deg, about Z), alpha(deg, about X), delta(mm, along Z), rho(mm, along X)]
and is applied as  Rx(alpha) * Rz(theta)  followed by a translation (rho, 0, delta),
exactly like `input_linkage_angles()` in the Dash app.

Pure Python (no numpy) so it runs inside FreeCAD's bundled interpreter too.
"""
import math

LINKAGE = [
    [0, 0, 0, 0, 0],
    [1, 0, 0, 54.5, 0],
    [1, 0, 0, 0, 0],      # theta_1  (row 2)
    [2, 0, 0, 84, 0],
    [2, 0, -90, 0, 0],
    [2, 0, 0, 61, 0],
    [2, 0, 180, 0, 0],
    [2, 90, 0, 0, 0],
    [2, 0, 0, 0, 0],      # theta_2  (row 8)
    [3, 0, -180, 0, 0],
    [3, 0, 0, 0, 70],
    [3, 0, 0, 0, 0],      # theta_3  (row 11)
    [4, 0, 0, -61, 0],
    [4, 0, 0, 0, 52],
    [4, 90, 0, 0, 0],
    [4, 0, 0, 0, 57],
    [4, 90, 0, 0, 0],
    [4, 0, 90, 0, 0],
    [4, 0, 0, 0, 0],      # theta_4  (row 18)
    [5, 90, 0, 0, 0],
    [5, 0, 0, 0, 47],
    [5, 0, 0, 44.5, 0],
    [5, -90, 0, 0, 0],
    [5, 0, -90, 0, 0],
    [5, -90, 0, 0, 0],
    [5, 0, 0, -47, 0],
    [5, 0, 0, 0, 0],      # theta_5  (row 26)
    [6, 0, 0, 0, 93],     # row 27: J6 frame / TCP at axis-6 shaft tip (was 135)
    [6, -90, 0, 0, 0],
    [6, 0, -90, 0, 0],
    [6, 0, 0, 0, 0],      # theta_6  (row 30)
    # [7, 0, 0, 0, 12],
    [7, 0, 90, 0, 0],
    [7, 90, 0, 0, 0],
    [7, 0, 90, 0, 0],
]
JOINT_ROWS = [2, 8, 11, 18, 26, 30]

# From "config - project Hayley V5.csv" (Absolute low/high angle limit), degrees
LIMITS_DEG = [(-100, 100), (-90, 90), (-90, 90), (-120, 120), (-100, 100), (-179.9, 179.9)]

# Joint speed model (deg/s) derived from Project_Hayley_ESP32_V20.ino:
#   FULL_SPEED 1000 steps/s, 200 steps/rev, 8 microsteps, gear_reduction 30:1 (axis 6 = 1:1)
#   -> 1000 / (200*8*30) * 360 = 7.5 deg/s (axes 1-5), 225 deg/s (axis 6)
STEPS_PER_S = 1000.0
GEAR = [30, 30, 30, 30, 30, 1]
JOINT_SPEED_DEG_S = [STEPS_PER_S / (200 * 8 * g) * 360.0 for g in GEAR]


def _mat(theta, alpha, delta, rho):
    t, a = math.radians(theta), math.radians(alpha)
    ct, st, ca, sa = math.cos(t), math.sin(t), math.cos(a), math.sin(a)
    return [[ct, -st, 0, rho],
            [st * ca, ct * ca, -sa, -sa * delta],
            [st * sa, ct * sa, ca, ca * delta],
            [0, 0, 0, 1]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def forward(joint_deg=(0, 0, 0, 0, 0, 0)):
    """Return (joint_frames, tcp) as 4x4 matrices in mm, base frame."""
    T = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
    frames = []
    for i, row in enumerate(LINKAGE):
        theta = row[1]
        if i in JOINT_ROWS:
            theta = joint_deg[JOINT_ROWS.index(i)]
        T = _mul(T, _mat(theta, row[2], 0, 0))
        if i in JOINT_ROWS:
            frames.append(T)
        T = _mul(T, _mat(0, 0, row[3], row[4]))
    return frames, T


def home_joints():
    """Joint origins (mm) and unit axes (base frame) at kinematic zero."""
    frames, tcp = forward()
    out = []
    for F in frames:
        out.append({
            "origin": [round(F[i][3], 6) for i in range(3)],
            "axis": [round(F[i][2], 6) + 0.0 for i in range(3)],
        })
    return out, tcp


if __name__ == "__main__":
    joints, tcp = home_joints()
    for i, j in enumerate(joints, 1):
        print("J%d origin=%s axis=%s" % (i, j["origin"], j["axis"]))
    print("TCP", [round(tcp[i][3], 2) for i in range(3)])
    print("speeds deg/s", JOINT_SPEED_DEG_S)
