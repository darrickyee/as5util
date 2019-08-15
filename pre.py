import pymel.core as pm

from .data import G8fMap, G8mMap, Ue4Map
from .utils import getPoleVector, orientJoint, getAverageLoc

FITSKEL_FILES = {
    'g8f': 'C:/Users/DSY/Documents/Maya/scripts/AdvancedSkeleton5Files/fitSkeletons/daz_g8f.ma',
    'g8m': 'C:/Users/DSY/Documents/Maya/scripts/AdvancedSkeleton5Files/fitSkeletons/daz_g8m.ma',
    'ue4': 'C:/Users/DSY/Documents/Maya/scripts/AdvancedSkeleton5Files/fitSkeletons/ue4.ma'
}
MAPS = {
    'g8f': G8fMap,
    'g8m': G8mMap,
    'ue4': Ue4Map
}


def preBuild(skel_map_name, load=True, fitskel_file=None):
    fitskel_file = fitskel_file or FITSKEL_FILES[skel_map_name]

    sk_map = getSkelMap(skel_map_name)
    custom_locs = sk_map.custom_locs

    # Load FitSkeleton
    if load:
        pm.importFile(fitskel_file)

    # Save custom translations
    custom_dict = dict()
    for joint in custom_locs:
        jnt_node = pm.ls(joint.split('_')[0], type='transform')
        if jnt_node:
            custom_dict[jnt_node[0]] = jnt_node[0].getTranslation()

    # Align FitSkeleton joints
    alignFitSkeleton(sk_map)

    # Custom Spine1_M and Knee translations
    jnt_node = pm.ls('Spine1')
    if jnt_node:
        pm.move(jnt_node[0],
                jnt_node[0].getParent().getTranslation(space='world'), pcp=True)
        pm.move(jnt_node[0], (0, 1, 0), r=True, ws=True, pcp=True)

    _alignKnee()

    # Re-apply custom translations
    for jnt_node in custom_dict:
        jnt_node.setTranslation(custom_dict[jnt_node])

    # Custom locations for breast
    for jnt_name in 'BreastMid', 'BreastEnd':
        jnt_loc = getAverageLoc(vtx
                                for vtx_list in pm.ls('Vtx'+jnt_name+'_R', r=True)[0].members()
                                for vtx in vtx_list)
        pm.move(jnt_name, jnt_loc)

    # Center mid joints
    for joint in [j for j in sk_map.joint_map if j[-2:] == '_M']:
        jnt_node = pm.ls(joint.split('_')[0], type='transform')
        if jnt_node:
            pm.move(jnt_node[0], 0, x=True, pcp=True)

    # Custom orientations
    _applyCustomOrients(('BreastBase', 'BreastMid'))

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
        jnt = pm.ls(joint.split('_')[0], type='transform')
        print('Aligning {0} to {1}'.format(jnt, tgt))
        if jnt and tgt:
            pm.move(jnt[0], tgt[0].getTranslation(ws=True), pcp=True)


def _alignKnee():
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


def _applyCustomOrients(joints):
    joints = pm.ls(joints)

    for jnt_node in joints:
        orientJoint(jnt_node,
                    jnt_node.listRelatives()[0],
                    up_vector=(0, 0, 1))
