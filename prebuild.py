import pymel.core as pm

from .maps import G8fMap, G8mMap, Ue4Map
from .utils import getPoleVector, orientJoint

FITSKEL_FILES = {
    'g8f': 'file_path',
    'g8m': 'file_path',
    'ue4': 'file_path'
}
MAPS = {
    'g8f': G8fMap,
    'g8m': G8mMap,
    'ue4': Ue4Map
}


def preBuild(skel_map_name):
    sk_map = getSkelMap(skel_map_name)
    custom_locs = sk_map.custom_xforms

    # Load FitSkeleton
    pm.importFile(FITSKEL_FILES[skel_map_name])

    # Align FitSkeleton joints
    alignFitSkeleton(sk_map)

    # Custom Spine1_M and Knee translations
    jnt_node = pm.ls('Spine1')
    if jnt_node:
        pm.move(jnt_node[0],
                jnt_node[0].getParent().getTranslation(space='world'), pcp=True)
        pm.move(jnt_node[0], (0, 1, 0), r=True, ws=True, pcp=True)

    alignKnee()

    # Custom translations
    for joint in custom_locs:
        jnt_node = pm.ls(joint.strip('_L').strip(
            '_M').strip('_R'), type='transform')
        if jnt_node:
            pm.move(jnt_node[0], jnt_node[0].getParent().getTranslation(
                space='world'), pcp=True)
            pm.move(jnt_node[0], custom_locs[joint], r=True, ws=True, pcp=True)

    # Center mid joints
    for joint in [j for j in sk_map.joint_map if j[-2:] == '_M']:
        jnt_node = pm.ls(joint.strip('_L').strip(
            '_M').strip('_R'), type='transform')
        if jnt_node:
            pm.move(jnt_node[0], 0, x=True, pcp=True)

    # Custom orientations
    for joint in 'BreastBase', 'BreastMid':
        jnt_node = pm.ls(joint)
        if jnt_node:
            orientJoint(jnt_node[0],
                        jnt_node[0].listRelatives()[0],
                        up_vector=(0, 0, 1))

    # Zero out end joint orientations
    end_joints = [jnt for jnt in pm.ls(
        sk_map.joint_map.keys()) if not jnt.listRelatives()]
    for jnt_node in end_joints:
        jnt_node.jointOrient.set((0, 0, 0))


def getSkelMap(skel_map_name):
    return MAPS[skel_map_name.lower()]()


def alignFitSkeleton(sk_map):
    joint_map = sk_map.joint_map

    for joint in joint_map:
        tgt = pm.ls(joint_map[joint], type='transform')
        jnt = pm.ls(joint.strip('_L').strip(
            '_M').strip('_R'), type='transform')

        if jnt and tgt:
            pm.move(jnt[0], tgt[0].getTranslation(ws=True), pcp=True)


def alignKnee():
    joint = pm.ls('Knee')
    if joint:
        curr_vec = getPoleVector(*pm.ls(['Hip', 'Knee', 'Ankle']))
        curr_vec[1] = 0
        curr_len = curr_vec.length()

        tgt_vec = getPoleVector(*pm.ls(['Hip', 'Ankle', 'Toes']))
        tgt_vec[1] = 0
        tgt_vec = -tgt_vec.normal()

        diff_vec = tgt_vec*curr_len - curr_vec

        pm.move(joint[0], diff_vec, r=True, ws=True, pcp=True)
