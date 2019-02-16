import pymel.core as pm
from AutoRig.rig.utils import getPoleVector, orientJoint, createControlCurve
import sys
sys.path.append('C:/Users/DSY/Documents/Maya/2018/scripts/AutoRig')


DAZ_MAP = {'Root_M': 'hip',
           'Pelvis_M': 'pelvis',
           'ButtockBase_R': '',
           'ButtockEnd_R': '',
           'Spine1_M': 'abdomenLower',
           'Spine2_M': 'abdomenUpper',
           'Spine3_M': 'chestLower',
           'Chest_M': 'chestUpper',
           'Neck_M': 'neckLower',
           'Head_M': 'head',
           'HeadEnd_M': '',
           'Jaw_M': 'lowerJaw',
           'JawEnd_M': 'Chin',
           'Tongue1_M': 'tongue01',
           'Tongue2_M': 'tongue02',
           'Tongue3_M': 'tongue03',
           'Tongue4_M': 'tongue04',
           'TongueEnd_M': '',
           'Hip_R': 'rThighBend',
           'Knee_R': 'rShin',
           'Ankle_R': 'rFoot',
           'Heel_R': '',
           'Toes_R': 'rToe',
           'FootSideInner_R': '',
           'FootSideOuter_R': '',
           'ToesEnd_R': '',
           'BreastBase_R': 'rPectoral',
           'BreastMid_R': '',
           'BreastEnd_R': '',
           'Scapula_R': 'rCollar',
           'Shoulder_R': 'rShldrBend',
           'Elbow_R': 'rForearmBend',
           'Wrist_R': 'rHand',
           'Cup_R': 'rHand',
           'ThumbFinger1_R': 'rThumb1',
           'ThumbFinger2_R': 'rThumb2',
           'ThumbFinger3_R': 'rThumb3',
           'ThumbFinger4_R': '',
           'IndexFinger1_R': 'rIndex1',
           'IndexFinger2_R': 'rIndex2',
           'IndexFinger3_R': 'rIndex3',
           'IndexFinger4_R': '',
           'MiddleFinger1_R': 'rMid1',
           'MiddleFinger2_R': 'rMid2',
           'MiddleFinger3_R': 'rMid3',
           'MiddleFinger4_R': '',
           'RingFinger1_R': 'rRing1',
           'RingFinger2_R': 'rRing2',
           'RingFinger3_R': 'rRing3',
           'RingFinger4_R': '',
           'PinkyFinger1_R': 'rPinky1',
           'PinkyFinger2_R': 'rPinky2',
           'PinkyFinger3_R': 'rPinky3',
           'PinkyFinger4_R': '',
           'Eye_R': 'rEye',
           'EyeEnd_R': ''
           }

JOINTS = {jnt: pm.ls(jnt[:-2])[0] for jnt in DAZ_MAP}

MANUAL_XFORMS = {'BreastEnd_R': (-0.7507, 0.0609, 2.0745),
                 'BreastMid_R': (-5.4764, 4.5825, 15.071),
                 'ButtockBase_R': (-6.7316, -11.1316, -7.9212),
                 'ButtockEnd_R': (0.0, 0.0202, -4.462),
                 'EyeEnd_R': (-0.0, 0.0, 1.9899),
                 'FootSideInner_R': (4.1749, -1.5189, 2.0264),
                 'FootSideOuter_R': (-4.1465, -1.5189, -1.6646),
                 'HeadEnd_M': (-0.0002, 19.7225, -0.0),
                 'Heel_R': (1.591, -6.8412, -6.9217),
                 'IndexFinger4_R': (-1.1097, -1.6503, 0.097),
                 'MiddleFinger4_R': (-1.2055, -1.7125, -0.0127),
                 'PinkyFinger4_R': (-1.0251, -1.3298, -0.1708),
                 'RingFinger4_R': (-1.1531, -1.7006, -0.0962),
                 'ThumbFinger4_R': (-0.7817, -2.2185, 1.468),
                 'ToesEnd_R': (-1.7017, -1.5189, 4.9737),
                 'TongueEnd_M': (0.0, 0.0426, 1.4055)}

MAP_JOINTS = [jnt for jnt in DAZ_MAP if DAZ_MAP[jnt]]
UNMAP_JOINTS = [jnt for jnt in DAZ_MAP if not DAZ_MAP[jnt]]


def preBuild():

    for jnt in MAP_JOINTS:
        joint = JOINTS[jnt]

        tgt_jnt = pm.ls(DAZ_MAP[jnt])[0]
        pm.move(joint, tgt_jnt.getTranslation(
            space='world'), ws=True, pcp=True)

    for jnt in ['Spine1_M', 'Knee_R']:
        joint = JOINTS[jnt]
        # Manual transform for spine base
        if jnt == 'Spine1_M':
            pm.move(joint, joint.getParent().getTranslation(
                space='world'), pcp=True)
            pm.move(joint, (0, 1, 0), r=True, ws=True, pcp=True)

        # Manual transform for knee
        if jnt == 'Knee_R':
            curr_vec = getPoleVector(*pm.ls(['Hip', 'Knee', 'Ankle']))
            tgt_vec = getPoleVector(*pm.ls(['Hip', 'Ankle', 'Toes']))
            diff_vec = -tgt_vec - curr_vec

            pm.move(joint, (diff_vec[0], 0, diff_vec[-1]),
                    r=True, ws=True, pcp=True)

    for jnt in MANUAL_XFORMS:
        joint = JOINTS[jnt]
        pm.move(joint, joint.getParent().getTranslation(
                space='world'), pcp=True)
        pm.move(joint, MANUAL_XFORMS[jnt], r=True, ws=True, pcp=True)

    # Center mid joints
    for jnt in [j for j in JOINTS if j[-2:] == '_M']:

        pm.move(JOINTS[jnt], 0, x=True, pcp=True)

    # Custom orientations
    for jnt in ['BreastBase_R', 'BreastMid_R']:
        joint = JOINTS[jnt]
        orientJoint(joint, joint.listRelatives()[0], up_vector=(0, 0, 1))

    # Zero out end joint orientations
    for joint in [jnt for jnt in JOINTS.values() if not jnt.listRelatives()]:
        joint.jointOrient.set((0, 0, 0))


def postBuild():
    # POST BUILD
    # Set control colors and shapes
    pm.mel.source(
        "C:/Users/DSY/Documents/Maya/2018/scripts/AutoRig/AS5_SetControls.mel")

    # Add twist joints
    for jname in ['Hip', 'Shoulder', 'Elbow']:
        for side in ['_R', '_L']:

            start_joint = pm.ls(jname+side)[0]
            end_joint = start_joint.listRelatives()[0]
            joint = pm.createNode('joint', n="{0}Twist{1}".format(jname, side))

            joint.setParent(start_joint)
            pm.delete(pm.parentConstraint(start_joint, joint))
            pm.makeIdentity(joint)
            joint.jointOrient.set((0, 0, 0))

            joint.translateX.set(end_joint.translateX.get()/2.0)

    # Add clavicle IK

    SIDE_COLOR = {
        '_R': (1, 0, 0),
        '_L': (0, 0, 1)
    }

    for side in SIDE_COLOR:

        fkctrl = pm.ls('FKScapula'+side)[0]

        ctrl = createControlCurve(
            'IKScapula'+side, ctrl_type='ik', size=10.0, color=SIDE_COLOR[side])
        pm.delete(pm.parentConstraint('Shoulder'+side, ctrl))
        pm.delete(pm.orientConstraint('Scapula'+side, ctrl))

        aimdir = 1
        if side == '_L':
            aimdir = -1
            pm.rotate(ctrl, 180, x=True, r=True, os=True)

        ctrl_xform = pm.createNode('transform', n='IKOffsetScapula'+side)
        ctrl_xform.setParent(ctrl)
        pm.makeIdentity(ctrl_xform)
        ctrl_xform.setParent('FKParentConstraintToChest_M')
        ctrl.setParent(ctrl_xform)
        ctrl.rotate.set(lock=True)

        cstr = pm.aimConstraint(ctrl, fkctrl, aimVector=(aimdir, 0, 0), upVector=(0, 0, aimdir),
                                worldUpType='objectrotation', worldUpVector=(0, 0, 1), worldUpObject=ctrl)

        fkctrl.visibility.set(False)

        for node in [ctrl, ctrl_xform]:
            pm.ls('ControlSet')[0].add(node)
            pm.ls('AllSet')[0].add(node)

    # ADD TOE SWIVEL
    # REORIENT ARM/LEG IK
    for ctrljnts in [('IKArm_R', 'Wrist_R'),
                     ('IKArm_L', 'Wrist_L'),
                     ('IKLeg_R', 'Ankle_R'),
                     ('IKLeg_L', 'Ankle_L')]:
        orientIkControl(*pm.ls(ctrljnts))

    # Center pivots for FK/IK controls and set to IK
    NODES = [ctrl for ctrl in pm.ls(
        'FKIK*', et='transform') if pm.hasAttr(ctrl, 'FKIKBlend')]

    for ctrl in NODES:
        pm.xform(ctrl, cp=True)
        ctrl.FKIKBlend.set(10)

    for node in pm.ls('Root_M') + pm.ls('Root_M')[0].listRelatives(ad=True, type='joint'):
        pm.ls('DeformSet')[0].add(node)


def orientIkControl(ik_ctrl, joint):
    # NEED TO FIX UP CHILD ROTATE CONSTRAINTS
    ctrl = ik_ctrl
    joint = joint

    side = ik_ctrl.name()[-2:]

    ctrl_cvs = ctrl.getCVs(space='world')
    ctrl_childs = ctrl.listRelatives(type='transform')

    child_xf = pm.createNode('transform', n='IKOrigOffset'+side)
    offset_xf = pm.createNode('transform', n='IKOrientOffset'+side)

    offset_parent = ctrl.getParent().getParent()
    offset_childs = offset_parent.listRelatives(type='transform')

    pm.delete(pm.parentConstraint(ctrl, child_xf))
    for child in ctrl_childs:
        child.setParent(child_xf)

    for child in offset_childs:
        child.setParent(None)

    pm.delete(pm.parentConstraint(joint, offset_xf))
    offset_xf.setParent(offset_parent)

    for child in offset_childs:
        child.setParent(offset_xf)
        pm.makeIdentity(child)

    child_xf.setParent(ctrl)

    if 'Leg' in ik_ctrl.name():
        ctrl.setCVs(ctrl_cvs, space='world')
        ctrl.updateCurve()

    pm.select(ik_ctrl)


def FKIKMatch(fk_ctrls, ik_ctrls, switch_ctrl):
    pass
