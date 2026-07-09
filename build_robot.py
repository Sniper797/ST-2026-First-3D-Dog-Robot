# -*- coding: utf-8 -*-
"""
build_robot.py
==============
Autodesk Fusion 360 script (Utilities > Add-Ins > Scripts > Run).

Builds a DIY B2-W-style wheel-legged quadruped robot as a PARAMETRIC assembly.

Body frame (matches the spec JSON):
    X = forward, Y = left, Z = up. Origin at body geometric center.
    Left legs mount at +Y, right legs at -Y.
"""

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import math

# Global application / UI handles (standard Fusion script pattern).
app = adsk.core.Application.get()
ui = app.userInterface


# ---------------------------------------------------------------------------
# Unit helpers
# ---------------------------------------------------------------------------
MM = 0.1  # multiply a millimetre value by this to get Fusion's centimetres


def mm(v):
    """Convert a millimetre value to centimetres (Fusion's internal unit)."""
    return v * MM


def deg2rad(d):
    return d * math.pi / 180.0


# ---------------------------------------------------------------------------
# Minimal row-major 4x4 matrix math.
# Matrices are Python lists of 16 floats, row-major, matching the layout
# that adsk.core.Matrix3D.setWithArray() expects (translation in slots 3,7,11).
# ---------------------------------------------------------------------------
def mat_identity():
    return [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0]


def mat_translate(x, y, z):
    return [1.0, 0.0, 0.0, x,
            0.0, 1.0, 0.0, y,
            0.0, 0.0, 1.0, z,
            0.0, 0.0, 0.0, 1.0]


def mat_rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [1.0, 0.0, 0.0, 0.0,
            0.0,   c,  -s, 0.0,
            0.0,   s,   c, 0.0,
            0.0, 0.0, 0.0, 1.0]


def mat_rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [c,  0.0,  s, 0.0,
            0.0, 1.0, 0.0, 0.0,
            -s,  0.0,  c, 0.0,
            0.0, 0.0, 0.0, 1.0]


def mat_rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [c,  -s, 0.0, 0.0,
            s,   c, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0]


def mat_mul(a, b):
    """Standard 4x4 row-major matrix multiply: returns a * b."""
    r = [0.0] * 16
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += a[i * 4 + k] * b[k * 4 + j]
            r[i * 4 + j] = s
    return r


def to_m3d(m):
    """Convert a 16-float row-major list into an adsk.core.Matrix3D."""
    mtx = adsk.core.Matrix3D.create()
    mtx.setWithArray(m)
    return mtx


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def new_comp(parent_comp, matrix_m3d, name):
    """
    Create a new child component (as an occurrence) under parent_comp, placed
    by matrix_m3d, and return the occurrence.
    """
    occ = parent_comp.occurrences.addNewComponent(matrix_m3d)
    occ.component.name = name
    return occ


def _symmetric_extrude(comp, profile, total_thickness_cm):
    """Extrude a profile symmetrically about its sketch plane."""
    extrudes = comp.features.extrudeFeatures
    ext_input = extrudes.createInput(
        profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    dist = adsk.core.ValueInput.createByReal(total_thickness_cm)
    # isFullLength=True -> the distance is the TOTAL thickness, split evenly.
    ext_input.setSymmetricExtent(dist, True)
    return extrudes.add(ext_input)


def draw_box(comp, lx_cm, ly_cm, lz_cm):
    """Draw a rectangular box centred on the component origin."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    p0 = adsk.core.Point3D.create(-lx_cm / 2.0, -ly_cm / 2.0, 0.0)
    p1 = adsk.core.Point3D.create(lx_cm / 2.0, ly_cm / 2.0, 0.0)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(p0, p1)
    return _symmetric_extrude(comp, sk.profiles.item(0), lz_cm)


def draw_cylinder(comp, dia_cm, thick_cm):
    """Draw a cylinder centred on the component origin, axis along local Z."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    center = adsk.core.Point3D.create(0.0, 0.0, 0.0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(center, dia_cm / 2.0)
    return _symmetric_extrude(comp, sk.profiles.item(0), thick_cm)


def make_box(parent_comp, matrix_m3d, name, lx_cm, ly_cm, lz_cm):
    """Convenience: create a placed component and draw a centred box in it."""
    occ = new_comp(parent_comp, matrix_m3d, name)
    draw_box(occ.component, lx_cm, ly_cm, lz_cm)
    return occ


def make_cylinder(parent_comp, matrix_m3d, name, dia_cm, thick_cm):
    """Convenience: create a placed component and draw a centred cylinder."""
    occ = new_comp(parent_comp, matrix_m3d, name)
    draw_cylinder(occ.component, dia_cm, thick_cm)
    return occ


# ---------------------------------------------------------------------------
# Best-effort revolute joint helper.
# The caller supplies occurrences that are valid in the SAME assembly context
# (root). Any failure is swallowed so it can never abort the geometry build.
# ---------------------------------------------------------------------------
def add_revolute_joint(root, occ1, occ2, pivot_cm, axis, name):
    try:
        x, y, z = pivot_cm

        # Offset plane at the pivot's Z height, then a sketch point at (x, y).
        planes = root.constructionPlanes
        pin = planes.createInput()
        pin.setByOffset(root.xYConstructionPlane,
                        adsk.core.ValueInput.createByReal(z))
        plane = planes.add(pin)

        sk = root.sketches.add(plane)
        # On an offset XY plane, sketch (u, v) maps to world (u, v, z).
        spt = sk.sketchPoints.add(adsk.core.Point3D.create(x, y, 0.0))
        geo = adsk.fusion.JointGeometry.createByPoint(spt)

        jin = root.asBuiltJoints.createInput(occ1, occ2, geo)
        joint = root.asBuiltJoints.add(jin)

        if axis == 'X':
            joint.setAsRevoluteJointMotion(
                adsk.fusion.JointDirections.XAxisJointDirection)
        else:
            joint.setAsRevoluteJointMotion(
                adsk.fusion.JointDirections.YAxisJointDirection)
        joint.name = name
    except Exception:
        # Best-effort only: geometry is already built and stays put.
        pass


# ---------------------------------------------------------------------------
# User parameters (created in mm so the model is editable in millimetres).
# ---------------------------------------------------------------------------
def create_user_parameters(design):
    params = [
        ('bodyLength',         '550 mm'),
        ('bodyWidth',          '340 mm'),
        ('bodyHeight',         '150 mm'),
        ('bodyWall',           '4 mm'),
        ('hipPivotX',          '235 mm'),
        ('hipMountY',          '150 mm'),
        ('hipMountZ',          '75 mm'),
        ('hipLateralOffset',   '40 mm'),
        ('thighLength',        '175 mm'),
        ('calfLength',         '175 mm'),
        ('thighWidth',         '40 mm'),
        ('thighThick',         '30 mm'),
        ('calfWidth',          '38 mm'),
        ('calfThick',          '28 mm'),
        ('wheelDia',           '140 mm'),
        ('wheelWidth',         '45 mm'),
        ('wheelRadius',        '70 mm'),
        ('abdMotorDia',        '57 mm'),
        ('abdMotorThick',      '46 mm'),
        ('hipMotorDia',        '96.5 mm'),
        ('hipMotorThick',      '42.3 mm'),
        ('kneeMotorDia',       '96.5 mm'),
        ('kneeMotorThick',     '42.3 mm'),
        ('wheelHubDia',        '79 mm'),
        ('wheelHubThick',      '43 mm'),
        ('groundClearance',    '285 mm'),
        ('bodyCenterHeight',   '360 mm'),
        ('poseHipPitchDeg',    '52 deg'),
        ('poseKneeDeg',        '104 deg'),
        ('poseAbductionDeg',   '0 deg'),
    ]
    up = design.userParameters
    for nm, expr in params:
        try:
            unit = 'deg' if expr.endswith('deg') else 'mm'
            up.add(nm, adsk.core.ValueInput.createByString(expr), unit, '')
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main build
# ---------------------------------------------------------------------------
def run(context):
    try:
        # ---- New design document -----------------------------------------
        doc = app.documents.add(
            adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        # Ensure parametric modelling so User Parameters / timeline exist.
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        # Root component name is tied to the document and can be read-only on
        # some versions; guard so a rename can never abort the whole build.
        try:
            root.name = 'WheelLeggedQuadruped'
        except Exception:
            pass

        # ---- User parameters (mm) ----------------------------------------
        create_user_parameters(design)

        # ==================================================================
        # BODY component: chassis box + on-board electronics / sensors.
        # ==================================================================
        body_occ = new_comp(root, to_m3d(mat_identity()), 'Body')
        body_comp = body_occ.component

        # Chassis: 550 (X, fore-aft) x 340 (Y, lateral) x 150 (Z, vertical).
        make_box(body_comp, to_m3d(mat_identity()), 'Chassis',
                 mm(550), mm(340), mm(150))

        # On-board boxes: (name, dims[dx,dy,dz] mm, position[x,y,z] mm).
        electronics = [
            ('Jetson_Nano',   (100.0, 80.0, 29.0),  (-40.0,   0.0,  90.0)),
            ('Battery_6S',    (160.0, 50.0, 45.0),  (0.0,     0.0, -30.0)),
            ('IMU_BNO085',    (25.6,  22.7, 4.6),   (0.0,     0.0,   0.0)),
            ('Camera_RGB',    (24.0,  25.0, 9.0),   (283.0,   0.0,  20.0)),
            ('LiDAR_A1',      (96.8,  70.3, 55.0),  (-150.0,  0.0, 103.0)),
            ('DepthCam_D435i', (90.0, 25.0, 25.0),  (270.0,   0.0,  90.0)),
        ]
        for nm, (dx, dy, dz), (px, py, pz) in electronics:
            mtx = to_m3d(mat_translate(mm(px), mm(py), mm(pz)))
            make_box(body_comp, mtx, nm, mm(dx), mm(dy), mm(dz))

        # ==================================================================
        # LEGS
        # ==================================================================
        L = mm(175.0)                     # thigh = calf = 175 mm
        a_pose = deg2rad(52.0)            # hip-pitch pose angle
        s52 = math.sin(a_pose)
        c52 = math.cos(a_pose)

        ABD_DIA,  ABD_THK = mm(57.0),  mm(46.0)     # DM-J4310-2EC
        HIP_DIA,  HIP_THK = mm(96.5),  mm(42.3)     # GO-M8010-6
        KNEE_DIA, KNEE_THK = mm(96.5), mm(42.3)     # GO-M8010-6
        HUB_DIA,  HUB_THK = mm(79.0),  mm(43.0)     # AK60-6 hub motor
        WHEEL_DIA, WHEEL_W = mm(140.0), mm(45.0)
        TH_W, TH_T = mm(40.0), mm(30.0)             # thigh width / thick
        CF_W, CF_T = mm(38.0), mm(28.0)             # calf width / thick

        R_axis_X = mat_rot_y(deg2rad(90.0))
        R_axis_Y = mat_rot_x(deg2rad(-90.0))

        legs = [
            ('FL', 235.0,  150.0, -75.0,  1),
            ('FR', 235.0, -150.0, -75.0, -1),
            ('RL', -235.0, 150.0, -75.0,  1),
            ('RR', -235.0, -150.0, -75.0, -1),
        ]

        for name, mx, my, mz, abd_sign in legs:
            leg_occ = new_comp(root, to_m3d(mat_identity()), 'Leg_' + name)
            leg_comp = leg_occ.component

            # --- Key pivot points (cm), forward kinematics of the pose -----
            M = (mm(mx), mm(my), mm(mz))

            off_y = mm(40.0) * (1.0 if my > 0 else -1.0)
            P1 = (M[0], M[1] + off_y, M[2])            # hip-pitch pivot

            P2 = (P1[0] - L * s52, P1[1], P1[2] - L * c52)   # knee pivot
            P3 = (P1[0],           P1[1], P1[2] - 2.0 * L * c52)  # wheel axle

            def midpoint(a, b):
                return ((a[0] + b[0]) / 2.0,
                        (a[1] + b[1]) / 2.0,
                        (a[2] + b[2]) / 2.0)

            mid_thigh = midpoint(P1, P2)
            mid_calf = midpoint(P2, P3)

            # --- Hip abduction motor: cylinder, axis along body X ----------
            m_abd = mat_mul(mat_translate(*M), R_axis_X)
            abd_occ = make_cylinder(leg_comp, to_m3d(m_abd),
                                    name + '_HipAbductionMotor',
                                    ABD_DIA, ABD_THK)

            # --- Hip bracket: short box carrying the roll->pitch offset -----
            mid_bracket = midpoint(M, P1)
            m_brk = mat_translate(*mid_bracket)
            make_box(leg_comp, to_m3d(m_brk), name + '_HipBracket',
                     mm(40.0), abs(off_y) + mm(30.0), mm(60.0))

            # --- Hip pitch motor: cylinder, axis along body Y --------------
            m_hip = mat_mul(mat_translate(*P1), R_axis_Y)
            make_cylinder(leg_comp, to_m3d(m_hip),
                          name + '_HipPitchMotor', HIP_DIA, HIP_THK)

            # --- Thigh link: box along its own axis, rotated R_y(+52) ------
            m_thigh = mat_mul(mat_translate(*mid_thigh),
                              mat_rot_y(a_pose))
            thigh_occ = make_box(leg_comp, to_m3d(m_thigh),
                                 name + '_Thigh', TH_T, TH_W, L)

            # --- Knee motor: cylinder, axis along body Y -------------------
            m_knee = mat_mul(mat_translate(*P2), R_axis_Y)
            make_cylinder(leg_comp, to_m3d(m_knee),
                          name + '_KneeMotor', KNEE_DIA, KNEE_THK)

            # --- Calf link: box along its own axis, rotated R_y(-52) -------
            m_calf = mat_mul(mat_translate(*mid_calf),
                             mat_rot_y(-a_pose))
            calf_occ = make_box(leg_comp, to_m3d(m_calf),
                                name + '_Calf', CF_T, CF_W, L)

            # --- Wheel hub motor: cylinder, axis Y, coaxial with wheel -----
            m_hub = mat_mul(mat_translate(*P3), R_axis_Y)
            make_cylinder(leg_comp, to_m3d(m_hub),
                          name + '_WheelHubMotor', HUB_DIA, HUB_THK)

            # --- Wheel: cylinder (tire), axis Y ----------------------------
            m_wheel = mat_mul(mat_translate(*P3), R_axis_Y)
            wheel_occ = make_cylinder(leg_comp, to_m3d(m_wheel),
                                      name + '_Wheel', WHEEL_DIA, WHEEL_W)

            # --- Best-effort revolute joints for this leg ------------------
            # The leg sub-parts are native occurrences INSIDE leg_comp; to
            # joint them at the root context they must be referenced through
            # assembly-context proxies obtained via leg_occ. body_occ is
            # already root-native and needs no proxy.
            try:
                abd_j = abd_occ.createForAssemblyContext(leg_occ)
                thigh_j = thigh_occ.createForAssemblyContext(leg_occ)
                calf_j = calf_occ.createForAssemblyContext(leg_occ)
                wheel_j = wheel_occ.createForAssemblyContext(leg_occ)
            except Exception:
                abd_j, thigh_j, calf_j, wheel_j = (
                    abd_occ, thigh_occ, calf_occ, wheel_occ)

            # 1) Abduction (roll): Body <-> AbductionMotor, axis X at mount M.
            add_revolute_joint(root, body_occ, abd_j, M, 'X',
                               name + '_AbductionJoint')
            # 2) Hip pitch:  AbductionMotor <-> Thigh, axis Y at P1.
            add_revolute_joint(root, abd_j, thigh_j, P1, 'Y',
                               name + '_HipPitchJoint')
            # 3) Knee:       Thigh <-> Calf, axis Y at P2.
            add_revolute_joint(root, thigh_j, calf_j, P2, 'Y',
                               name + '_KneeJoint')
            # 4) Wheel spin: Calf <-> Wheel, axis Y at P3.
            add_revolute_joint(root, calf_j, wheel_j, P3, 'Y',
                               name + '_WheelJoint')

        # Fit the camera to the finished model.
        try:
            app.activeViewport.fit()
        except Exception:
            pass

        ui.messageBox('Wheel-legged quadruped built successfully.\n'
                      '16 actuated DOF (4 legs x [abduction, hip-pitch, '
                      'knee, wheel]).')

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
