import pymel.core as pm
from .utils import getPoleVector, orientJoint, createControlCurve, lockAndHideAttrs
from .maps import G8fMap, G8mMap, Ue4Map
from .jcms import DRIVERS, createWtDrivers

SIDE_COLOR = {
    '_R': (1, 0, 0),
    '_L': (0, 0, 1)
}

MAPS = {
    'g8f': G8fMap,
    'g8m': G8mMap,
    'ue4': Ue4Map
}

CTRL_SHAPE_FILES = {
    'g8f': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/AS5Controls_G8F.mel",
    'g8m': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/AS5Controls_G8M.mel",
    'ue4': "C:/Users/DSY/Documents/Maya/2018/scripts/as5util/AS5Controls_UE4.mel"
}

MESH_FILES = {
    'g8f': "C:/Users/DSY/Documents/Maya/projects/_UE4-Chars/scenes/Mesh/Ref/Mesh_G8F.ma",
    'g8m': "C:/Users/DSY/Documents/Maya/projects/_UE4-Chars/scenes/Mesh/Ref/Mesh_G8M.ma"
}

MAT_FILE = "C:/Users/DSY/Documents/Maya/projects/_UE4-Chars/assets/G8_MATS.ma"


def initMapper(skel_map_name):
    return MAPS[skel_map_name.lower()]()


def setupGeo(skel_map_name):
    pm.importFile(MESH_FILES[skel_map_name])
    pm.importFile(MAT_FILE)

    for ns in ['BodyGeo', 'HeadGeo']:
        pm.skinCluster('Root_M', '{0}:Mesh'.format(ns), skinMethod=1)
        addBlendShapes(ns)


def addBlendShapes(name_space):
    pm.select(pm.ls('{0}:Morphs'.format(name_space))
              [0].listRelatives(), r=True)
    pm.select('{0}:Mesh'.format(name_space), add=True)
    pm.blendShape(frontOfChain=1, n='Morphs{0}'.format(
        name_space.replace('Geo', '')))


def preBuild(skel_map_name):
    sk_map = initMapper(skel_map_name)
    sk_joints = sk_map.getJoints()
    sk_joint_map = sk_map.joint_map
    sk_mapped_joints = sk_map.jnames_mapped
    sk_xforms = sk_map.custom_xforms

    for jnt in sk_mapped_joints:
        joint = sk_joints[jnt]

        tgt_jnt = pm.ls(sk_joint_map[jnt])[0]
        pm.move(joint, tgt_jnt.getTranslation(
            space='world'), ws=True, pcp=True)

    for jnt in ['Spine1_M', 'Knee_R']:
        joint = sk_joints[jnt]

        # Manual transform for spine base
        if jnt == 'Spine1_M':
            pm.move(joint, joint.getParent().getTranslation(
                space='world'), pcp=True)
            pm.move(joint, (0, 1, 0), r=True, ws=True, pcp=True)

        # Manual transform for knee
        if jnt == 'Knee_R':

            curr_vec = getPoleVector(*pm.ls(['Hip', 'Knee', 'Ankle']))
            curr_vec[1] = 0
            curr_len = curr_vec.length()

            tgt_vec = getPoleVector(*pm.ls(['Hip', 'Ankle', 'Toes']))
            tgt_vec[1] = 0
            tgt_vec = -tgt_vec.normal()

            diff_vec = tgt_vec*curr_len - curr_vec

            pm.move(joint, diff_vec, r=True, ws=True, pcp=True)

    # Custom transforms
    for jnt in sk_xforms:
        joint = sk_joints[jnt]
        pm.move(joint, joint.getParent().getTranslation(
            space='world'), pcp=True)
        pm.move(joint, sk_xforms[jnt], r=True, ws=True, pcp=True)

    # Center mid joints
    for jnt in [j for j in sk_joints if j[-2:] == '_M']:

        pm.move(sk_joints[jnt], 0, x=True, pcp=True)

    # Custom orientations
    for jnt in ['BreastBase_R', 'BreastMid_R']:
        joint = sk_joints.get(jnt, None)
        if joint:
            orientJoint(joint, joint.listRelatives()[0], up_vector=(0, 0, 1))

    # Zero out end joint orientations
    for joint in [jnt for jnt in sk_joints.values() if not jnt.listRelatives()]:
        joint.jointOrient.set((0, 0, 0))


def postBuild(skel_map_name):
    # POST BUILD
    obj_sets = {set_name: pm.ls(set_name)[0] for set_name in [
        'AllSet', 'ControlSet', 'DeformSet']}

    twist_joints = postAddTwistJoints()
    for set_name in ['AllSet', 'DeformSet']:
        obj_sets[set_name].addMembers(twist_joints)

    ik_clav = postAddClavicleIK()
    for set_name in ['AllSet', 'ControlSet']:
        obj_sets[set_name].addMembers(ik_clav)

    # ADD TOE SWIVEL
    # REORIENT ARM/LEG IK
    for ctrljnts in [('IKArm_R', 'Wrist_R'),
                     ('IKArm_L', 'Wrist_L'),
                     ('IKLeg_R', 'Ankle_R'),
                     ('IKLeg_L', 'Ankle_L')]:
        postOrientIkControl(*pm.ls(ctrljnts))

    # Center pivots for FK/IK controls and set to IK
    nodes = [ctrl for ctrl in pm.ls(
        'FKIK*', et='transform') if pm.hasAttr(ctrl, 'FKIKBlend')]

    for ctrl in nodes:
        pm.xform(ctrl, cp=True)
        ctrl.FKIKBlend.set(10)

    pm.ls('IKSpine3_M')[0].stretchy.set(0)
    pm.ls('IKSpline3_M')[0].volume.set(0)

    # Add UE4 IK joints
    postAddUe4Joints()

    # Update sets
    postUpdateSets(obj_sets)

    # Lock/hide scale for all controls
    lockAndHideAttrs([ctrl for ctrl in obj_sets['ControlSet']],
                     attr_list=['scaleX', 'scaleY', 'scaleZ'])
    # Lock translate for FK controls
    lockAndHideAttrs([ctrl for ctrl in obj_sets['ControlSet'] if ctrl.name()[
                     :2] == 'FK'], attr_list=['translateX', 'translateY', 'translateZ'])

    # Update control colors/shapes
    setControlShapes(skel_map_name)


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

    for ctrl in ['AimEye_M'] + [cname+side
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

    pm.delete('PoleOffset'+limb+side+'_parentConstraint1')

    pm.select(ik_ctrl)


def postAddWeightDrivers():
    for bs_name in ['MorphsHead', 'MorphsBody']:
        for driver in DRIVERS:
            createWtDrivers(bs_name, morph_name=driver, **DRIVERS[driver])

    wd_grp = pm.createNode('transform', n='WeightDrivers')
    for wt_drv in pm.ls('wtDrv*', et='transform'):
        wt_drv.setParent(wd_grp)
    wd_grp.visibility.set(False)
    wd_grp.setParent('Main')
