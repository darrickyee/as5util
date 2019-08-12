import pymel.core as pm
from spaceSwitchTool import spaceSwitchSetup as switchSetup
from .data import getSpaceSwitchArgs, SPACE_LIST
from .utils import createControlCurve, lockAndHideAttrs


SIDE_COLOR = {
    '_R': (1, 0, 0),
    '_L': (0, 0, 1)
}

CTRL_SHAPE_FILES = {
    'g8f': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/data/ctrls_g8f.mel",
    'g8m': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/data/ctrls_g8m.mel",
    'ue4': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/data/ctrls_ue4.mel"
}


def postBuild(skel_map_name):

    obj_sets = {set_name: pm.ls(set_name)[0] for set_name in [
        'AllSet', 'ControlSet', 'DeformSet']}

    # Add twist joints
    twist_joints = postAddTwistJoints()
    for set_name in ['AllSet', 'DeformSet']:
        obj_sets[set_name].addMembers(twist_joints)

    # Add IK Clavicle
    ik_clav = postAddClavicleIK()
    for set_name in ['AllSet', 'ControlSet']:
        obj_sets[set_name].addMembers(ik_clav)

    # ADD TOE SWIVEL
    # Reorient Arm/Leg IK
    for ctrljnts in [('IKArm_R', 'Wrist_R'),
                     ('IKArm_L', 'Wrist_L'),
                     ('IKLeg_R', 'Ankle_R'),
                     ('IKLeg_L', 'Ankle_L')]:
        postOrientIkControl(*pm.ls(ctrljnts))

    # Center pivots for FK/IK controls and set to IK
    fkik_nodes = [ctrl for ctrl in pm.ls(
        'FKIK*', et='transform') if pm.hasAttr(ctrl, 'FKIKBlend')]
    for ctrl in fkik_nodes:
        pm.xform(ctrl, cp=True)
        ctrl.FKIKBlend.set(10)

    # Set default properties
    pm.ls('IKSpine3_M')[0].stretchy.set(0)
    pm.ls('IKSpine3_M')[0].volume.set(0)
    pm.ls('IKSpline3_M')[0].volume.set(0)

    # Add UE4 IK joints
    postAddUe4Joints()

    # Update sets
    postUpdateSets(obj_sets)

    # Lock/hide scale for all controls
    lockAndHideAttrs([ctrl for ctrl in obj_sets['ControlSet']],
                     attr_list=('scaleX', 'scaleY', 'scaleZ'))
    # Lock translate for FK controls
    lockAndHideAttrs([ctrl for ctrl in obj_sets['ControlSet']
                      if ctrl.name()[:2] == 'FK'], attr_list=('translateX', 'translateY', 'translateZ'))

    # Update control colors/shapes
    setControlShapes(skel_map_name)

    # Hide joints, add to display layer
    postCreateJointLayer()

    # Breasts
    for xform in ('FKOffsetBreastBase'+side for side in ('_L', '_R')):
        pm.orientConstraint(('Chest_M', 'Spine3_M'), xform, mo=True)

    # Add spaces
    postAddSpaceSwitches(getSpaceSwitchArgs(SPACE_LIST))


def postAddSpaceSwitches(args_list):
    if pm.ls('SpaceSystem'):
        pm.delete('SpaceSystem')

    pm.createNode('transform', n='SpaceSystem').setParent('Main')

    ss_win = switchSetup.SpaceSwitchWindow()
    for arg_dict in args_list:
        ctrl_node = pm.ls(arg_dict['controller'])
        if not ctrl_node:
            raise ValueError("Controller '{}' not found.".format(
                arg_dict['controller']))

        pm.delete(ctrl_node[0].getParent().listRelatives(type='constraint'))

        setSpaceSwitchProperties(ss_win, arg_dict)

        ss_win.generateSpaces()


def setSpaceSwitchProperties(space_switch_window, arg_dict):
    # SpaceSwitch properties:
    # self.drivenNode, self.switchController, self.constraintType, spaces, self.spacesGroup, self.switchAttribute
    space_switch_window.clearSpacesFrame()

    cons_dict = {
        'parent': 0,
        'point': 1,
        'orient': 2
    }

    space_switch_window.drivenNodeEdit.setText(arg_dict['drivenNode'])
    space_switch_window.switchControllerEdit.setText(arg_dict['controller'])
    space_switch_window.constraintComboBox.setCurrentIndex(
        cons_dict.get(arg_dict['constraintType'], 0))
    space_switch_window.spacesEdit.setText(arg_dict['spacesGrp'])
    space_switch_window.attributeNameEdit.setText(arg_dict['attrName'])

    for space in arg_dict['driverSpaces']:
        new_space = space_switch_window.addSpaceLayout(False)
        new_space.driverNodeEdit.setText(space.keys()[0])
        new_space.displayNameEdit.setText(space.values()[0])

    return space_switch_window


def postUpdateSets(obj_set_dict):
    joints = pm.ls('Root_M') + \
        pm.ls('Root_M')[0].listRelatives(ad=True, type='joint')
    ctrls = [ctrl for ctrl in obj_set_dict['ControlSet']
             if ((ctrl.name()[:2] in ['FK', 'IK']) or
                 (ctrl.name()[:4] in ['AimE', 'Fing', 'Pole', 'Roll', 'Root'])) and
             'Extra' not in ctrl.name() and
             'cv' not in ctrl.name()]

    set_members = {'ControlSet': ctrls, 'DeformSet': joints}

    for set_name in ['ControlSet', 'DeformSet']:
        obj_set_dict[set_name].clear()
        obj_set_dict[set_name].addMembers(set_members[set_name])


def postAddUe4Joints():
    # Add UE4 IK joints
    jnt_list = list()

    for ctrl in ['AimEye_M', 'RootX_M'] + [cname+side
                                           for cname in ['IKArm', 'PoleArm', 'IKLeg', 'PoleLeg']
                                           for side in ['_R', '_L']]:

        jnt = pm.createNode('joint', n='CTRL'+ctrl)
        jnt.setParent(pm.ls('DeformationSystem')[0])
        pm.delete(pm.parentConstraint(pm.ls(ctrl)[0], jnt))
        pm.makeIdentity(jnt, apply=True)

        if 'IKArm' in ctrl:
            tgt = pm.ls('Wrist'+ctrl[-2:])[0]
        elif 'IKLeg' in ctrl:
            tgt = pm.ls('Ankle'+ctrl[-2:])[0]
        else:
            tgt = pm.ls(ctrl)[0]
        pm.parentConstraint(tgt, jnt)

        jnt_list.append(jnt)

    return jnt_list


def postAddTwistJoints():
    # Add twist joints
    jnt_list = list()

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

            jnt_list.append(joint)

    return jnt_list


def postAddClavicleIK():
    # Add clavicle IK
    ctrl_list = list()

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

        ctrl_list.append(ctrl)

    return ctrl_list


def setControlShapes(skel_map_name):
    # Set control colors and shapes
    pm.mel.source(CTRL_SHAPE_FILES[skel_map_name])
    # Center pivots for FKIK controls
    for ctrl in pm.ls(['FKIK{}*_*'.format(limb) for limb in ('Arm', 'Leg', 'Spine', 'Spline')], type='transform'):
        pm.xform(ctrl, cp=True)


def postOrientIkControl(ik_ctrl, joint):
    # NEED TO FIX UP CHILD ROTATE CONSTRAINTS

    if 'Arm' in ik_ctrl.name():
        limb = 'Arm'
    else:
        limb = 'Leg'

    side = ik_ctrl.name()[-2:]

    ctrl_cvs = ik_ctrl.getCVs(space='world')
    ctrl_childs = ik_ctrl.listRelatives(type='transform')

    child_xf = pm.createNode('transform', n='IKOrigOffset'+limb+side)
    offset_xf = pm.createNode('transform', n='IKOrientOffset'+limb+side)

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

    if pm.ls('PoleOffset'+limb+side+'_parentConstraint1'):
        pm.delete('PoleOffset'+limb+side+'_parentConstraint1')
        pm.parentConstraint('IK'+limb+side, 'PoleOffset' +
                            limb+side, mo=True, sr=('x', 'y', 'z'))

    pm.select(ik_ctrl)


def postCreateJointLayer():

    for joint in pm.ls(type='joint'):
        joint.drawStyle.set(2)

    deform_joints = pm.listRelatives(
        'DeformationSystem', ad=True, type='joint')
    for joint in deform_joints:
        joint.radius.set(10)
        joint.drawStyle.set(0)

    if not pm.ls('JointLayer'):
        layer = pm.createDisplayLayer(name='JointLayer')

    layer.addMembers(deform_joints)
    layer.displayType.set(2)
