import pymel.core as pm
from .utils import getPoleVector, orientJoint, createControlCurve, lockAndHideAttrs
from .maps import DazMap, Ue4Map

SIDE_COLOR = {
    '_R': (1, 0, 0),
    '_L': (0, 0, 1)
}

MAPS = {
    'daz': DazMap,
    'ue4': Ue4Map
}


def initMapper(skel_map_name):
    return MAPS[skel_map_name.lower()]()


SKMAP = initMapper('daz')
JOINT_MAP = SKMAP.joint_map
MAPPED_JOINTS = SKMAP.jnames_mapped
UNMAPPED_JOINTS = SKMAP.jnames_unmapped
XFORMS = SKMAP.custom_xforms


def preBuild():
    JOINTS = SKMAP.getJoints()

    for jnt in MAPPED_JOINTS:
        joint = JOINTS[jnt]

        tgt_jnt = pm.ls(JOINT_MAP[jnt])[0]
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

    for jnt in XFORMS:
        joint = JOINTS[jnt]
        pm.move(joint, joint.getParent().getTranslation(
                space='world'), pcp=True)
        pm.move(joint, XFORMS[jnt], r=True, ws=True, pcp=True)

    # Center mid joints
    for jnt in [j for j in JOINTS if j[-2:] == '_M']:

        pm.move(JOINTS[jnt], 0, x=True, pcp=True)

    # Custom orientations
    for jnt in ['BreastBase_R', 'BreastMid_R']:
        joint = JOINTS.get(jnt, None)
        if joint:
            orientJoint(joint, joint.listRelatives()[0], up_vector=(0, 0, 1))

    # Zero out end joint orientations
    for joint in [jnt for jnt in JOINTS.values() if not jnt.listRelatives()]:
        joint.jointOrient.set((0, 0, 0))


def postBuild():
    # POST BUILD
    control_set = pm.ls('ControlSet')[0]

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

        lockAndHideAttrs([ctrl], attr_list=['rotateX', 'rotateY', 'rotateZ'])

        pm.aimConstraint(ctrl, fkctrl, aimVector=(aimdir, 0, 0), upVector=(0, 0, aimdir),
                         worldUpType='objectrotation', worldUpVector=(0, 0, 1), worldUpObject=ctrl)

        fkctrl.visibility.set(False)

        control_set.add(ctrl)
        pm.ls('AllSet')[0].add(ctrl)

    # ADD TOE SWIVEL
    # REORIENT ARM/LEG IK
    for ctrljnts in [('IKArm_R', 'Wrist_R'),
                     ('IKArm_L', 'Wrist_L'),
                     ('IKLeg_R', 'Ankle_R'),
                     ('IKLeg_L', 'Ankle_L')]:
        orientIkControl(*pm.ls(ctrljnts))

    # Center pivots for FK/IK controls and set to IK
    nodes = [ctrl for ctrl in pm.ls(
        'FKIK*', et='transform') if pm.hasAttr(ctrl, 'FKIKBlend')]

    # Set controls to IK
    for ctrl in nodes:
        pm.xform(ctrl, cp=True)
        ctrl.FKIKBlend.set(10)

    # Add joints to deform set
    for node in pm.ls('Root_M') + pm.ls('Root_M')[0].listRelatives(ad=True, type='joint'):
        pm.ls('DeformSet')[0].add(node)

    # Lock/hide scale for all controls
    lockAndHideAttrs([ctrl for ctrl in control_set],
                     attr_list=['scaleX', 'scaleY', 'scaleZ'])
    # Lock translate for FK controls
    lockAndHideAttrs([ctrl for ctrl in control_set if ctrl.name()[
                     :2] == 'FK'], attr_list=['translateX', 'translateY', 'translateZ'])

    # Re-populate control set
    ctrl_curves = [ctrl for ctrl in control_set
                   if ((ctrl.name()[:2] in ['FK', 'IK']) or
                       (ctrl.name()[:4] in ['AimE', 'Fing', 'Pole', 'Roll', 'Root'])) and
                   'Extra' not in ctrl.name() and
                   'cv' not in ctrl.name()]
    control_set.clear()
    control_set.addMembers(ctrl_curves)


def orientIkControl(ik_ctrl, joint):
    # NEED TO FIX UP CHILD ROTATE CONSTRAINTS

    side = ik_ctrl.name()[-2:]

    ctrl_cvs = ik_ctrl.getCVs(space='world')
    ctrl_childs = ik_ctrl.listRelatives(type='transform')

    child_xf = pm.createNode('transform', n='IKOrigOffset'+side)
    offset_xf = pm.createNode('transform', n='IKOrientOffset'+side)

    offset_parent = ik_ctrl.getParent().getParent()
    offset_childs = offset_parent.listRelatives(type='transform')

    pm.delete(pm.parentConstraint(ik_ctrl, child_xf))
    for child in ctrl_childs:
        child.setParent(child_xf)

    for child in offset_childs:
        child.setParent(None)

    pm.delete(pm.parentConstraint(joint, offset_xf))
    offset_xf.setParent(offset_parent)

    for child in offset_childs:
        child.setParent(offset_xf)
        pm.makeIdentity(child)

    child_xf.setParent(ik_ctrl)

    if 'Leg' in ik_ctrl.name():
        ik_ctrl.setCVs(ctrl_cvs, space='world')
        ik_ctrl.updateCurve()

    # Set control colors and shapes
    if SKMAP.__class__.__name__ == 'DazMap':
        pm.mel.source(
            "C:/Users/DSY/Documents/Maya/2018/scripts/AutoRig/AS5_SetControls.mel")

    if SKMAP.__class__.__name__ == 'Ue4Map':
        pm.mel.source(
            "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/AS5Controls_UE4.mel")

    pm.select(ik_ctrl)
