import pymel.core as pm
from .utils import getPoleVector, alignToWorldVector

CLIPBOARD = dict()


def setupHik():
    setTPose()
    _createHikPoleControls()


def setTPose():

    limb_ctrls = {
        'Arm': ('Shoulder', 'Elbow', 'Wrist'),
        'Leg': ('Hip', 'Knee', 'Ankle')
    }

    for side in '_L', '_R':

        sign = -1 if side == '_L' else 1
        limb_vecs = {
            'Arm': ((-1, 0, 0), (0, 0, sign)),
            'Leg': ((0, -sign, 0), (0, 0, sign))
        }

        # Move IKScapula controls
        ctrl, prnt = pm.ls('IKScapula'+side, 'Scapula'+side)
        ctrl_loc, prnt_loc = (xf.getTranslation(ws=True)
                              for xf in (ctrl, prnt))

        vec_len = (ctrl_loc - prnt_loc).length()
        vec_len = -vec_len if side == '_R' else vec_len
        offset_loc = pm.dt.Vector((vec_len, 0, 0)) + prnt_loc

        ctrl.setTranslation(offset_loc, ws=True)

        # Arms and Legs
        for limb in 'Arm', 'Leg':
            # Set to FK
            pm.setAttr('FKIK'+limb+side+'.FKIKBlend', 0)

            for ctrl in limb_ctrls[limb]:
                x_vec, y_vec = limb_vecs[limb]
                alignToWorldVector('FK'+ctrl+side, aim_x=x_vec, aim_y=y_vec)

            # Align IK
            alignIk2Fk('FK'+limb_ctrls[limb][0]+side)

            # Move pole vector controls
            if limb == 'Arm':
                pole_offset = (0, 0, -50)
            else:
                pole_offset = (0, 0, 50)

            pm.move('Pole'+limb+side,
                    pm.ls('Elbow'+side)[0].getTranslation(ws=True)+pole_offset)


def _createHikPoleControls():

    for limb in 'Arm', 'Leg':
        for side in '_L', '_R':
            hikloc = pm.spaceLocator(n='Pole'+limb+'HikLoc'+side)
            jnt_map = {
                'Arm': 'Elbow',
                'Leg': 'Knee'
            }
            pm.delete(pm.parentConstraint(jnt_map[limb]+side, hikloc))
            pm.parentConstraint(hikloc, 'PoleExtra'+limb+side, mo=True)


def zeroControls():
    ctrl_list = pm.ls(sl=True) or pm.ls('ControlSet')[0].members()
    for ctrl in ctrl_list:
        ctrl.rotate.set((0, 0, 0))
        if 'IK' in ctrl.name():
            ctrl.translate.set((0, 0, 0))


JNT_GRPS = {
    'Arm': ['Shoulder', 'Elbow', 'Wrist'],
    'Leg': ['Hip', 'Knee', 'Ankle', 'Toes']
}


def alignFk2Ik(ik_ctrl):
    ik_ctrl = pm.ls(ik_ctrl)[0]

    grp_name = None
    if 'Arm' in ik_ctrl.name():
        grp_name = 'Arm'
    if 'Leg' in ik_ctrl.name():
        grp_name = 'Leg'

    if not grp_name:
        return

    side = ik_ctrl.name()[-2:]

    jnts = pm.ls([jname+side for jname in JNT_GRPS[grp_name]])
    fk_ctrls = pm.ls(['FK'+jnt.name() for jnt in jnts])

    for i, fk in enumerate(fk_ctrls):
        fk.setRotation(jnts[i].getRotation(ws=True), ws=True)


def alignIk2Fk(fk_ctrl):
    fk_ctrl = pm.ls(fk_ctrl)[0]

    side = fk_ctrl.name()[-2:]
    grp_name = [grp for grp in JNT_GRPS.keys() if fk_ctrl.name()[
        2:-2] in JNT_GRPS[grp]][0]

    jnts = pm.ls([jname+side for jname in JNT_GRPS[grp_name]])

    jnt_chain = jnts[:3]

    ik_ctrl = pm.ls('IK'+grp_name+side)[0]
    ik_pole = pm.ls('Pole'+grp_name+side)[0]

    ik_ctrl.setTranslation(jnt_chain[-1].getTranslation(ws=True), ws=True)
    ik_ctrl.setRotation(jnt_chain[-1].getRotation(ws=True), ws=True)

    pole_offset = getPoleVector(*jnt_chain)
    ik_pole.setTranslation(jnt_chain[1].getTranslation(
        ws=True) + pole_offset*20.0, ws=True)

    if grp_name == 'Leg':
        # Get worldspace transform of Toe
        tgt_mat = pm.ls('Toes'+side)[0].getMatrix(ws=True)

        pm.setAttr('FKIK'+grp_name+side+'.FKIKBlend', 10)

        # Get offset of IK ctrl relative to Toe
        offset = pm.ls('IKToes'+side)[0].getMatrix(ws=True) * tgt_mat.inverse()
        # Compose target Toe rotation with offset rotation
        ctrl_mat = offset * tgt_mat
        # Set rotation
        pm.ls('IKToes'+side)[0].setRotation(ctrl_mat.rotate, ws=True)

        pm.setAttr('FKIK'+grp_name+side+'.FKIKBlend', 0)


def mirrorControls(center_xform=None):

    if not center_xform:
        center_xform = pm.ls('RootX_M', type='transform')
        center_xform = center_xform[0] if center_xform else None

    side_suffixes = ['_L', '_R']

    if not pm.ls(sl=True):
        return

    ctrls = [ctrl for ctrl in pm.ls(
        sl=True) if ctrl.name()[-2:] in side_suffixes]
    if not ctrls:
        pm.error('Invalid controls selected')

    ctr_node = pm.createNode('transform')
    if center_xform:
        pm.delete(pm.parentConstraint(center_xform, ctr_node))

    mir_xforms = _buildMirrorXforms(ctrls, ctr_node)

    tgt_ctrls = pm.ls([ctrl.name()[:-2] + side_suffixes[1 -
                                                        side_suffixes.index(ctrl.name()[-2:])] for ctrl in ctrls])

    constraint_list = list()

    for ctrl, xform in zip(tgt_ctrls, mir_xforms):
        if all(ctrl.getAttr(attr_name, lock=False, keyable=True) for attr_name in ['translateX', 'translateY', 'translateZ']):
            constraint_list.append(pm.pointConstraint(xform, ctrl))
        if all(ctrl.getAttr(attr_name, lock=False, keyable=True) for attr_name in ['rotateX', 'rotateY', 'rotateZ']):
            constraint_list.append(pm.orientConstraint(xform, ctrl))

    pm.delete(constraint_list)
    pm.delete(mir_xforms)
    pm.delete(ctr_node)


def _buildMirrorXforms(xform_list, center_xform=None):
    if not center_xform:
        center_xform = pm.createNode('transform')

    mir_xforms = [pm.createNode('transform') for _ in xform_list]

    for mir_xform, xform in zip(mir_xforms, xform_list):
        pm.delete(pm.parentConstraint(xform, mir_xform))
        mir_xform.setParent(center_xform)

    center_xform.scaleX.set(-1)

    return mir_xforms


def copyTransforms(ctrl):
    if not pm.ls(sl=True):
        pm.warning("Cannot copy: No node selected.")
        return

    ctrl = pm.ls(sl=True)[0]
    global CLIPBOARD

    CLIPBOARD['translate'] = ctrl.getTranslation(ws=True)
    CLIPBOARD['rotate'] = ctrl.getRotation(ws=True)


def pasteTransforms(ctrl):
    if not pm.ls(sl=True):
        pm.warning("Cannot copy: No node selected.")
        return

    ctrl = pm.ls(sl=True)[0]
    global CLIPBOARD

    if ctrl.translateX.get(lock=False, keyable=True) and CLIPBOARD.get('translate', None):
        ctrl.setTranslation(CLIPBOARD['translate'], ws=True)

    if ctrl.rotateX.get(lock=False, keyable=True) and CLIPBOARD.get('rotate', None):
        ctrl.setRotation(CLIPBOARD['rotate'], ws=True)


def copyAttrs(ctrl):
    global CLIPBOARD
    CLIPBOARD = dict()
    for attr in ctrl.listAttr(locked=False, keyable=True):
        CLIPBOARD[attr.shortName()] = attr.get()


def pasteAttrs(ctrl):
    global CLIPBOARD
    for attr_name in CLIPBOARD:
        if ctrl.hasAttr(attr_name):
            ctrl.setAttr(attr_name, CLIPBOARD[attr_name])
