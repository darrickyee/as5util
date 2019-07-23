import pymel.core as pm

DRIVERS = {
    'POS_MouthOpen': {
        'joint_base': 'Jaw',
        'rotation': (0, 0, 25),
        'keys': [(0.0, 0.0), (0.5, 1.0), (1.0, 1.0)],
    },
    'JCM_NeckBack_27': {
        'joint_base': 'Neck',
        'rotation': (0, 0, -17.5),
        'keys': [(0.0, 0.0), (13.5/17.5, 1.0), (1.0, 1.0)]
    },
    'JCM_NeckFwd_35': {
        'joint_base': 'Neck',
        'rotation': (0, 0, 22.5),
        'keys': [(0.0, 0.0), (17.5/22.5, 1.0), (1.0, 1.0)]
    },
    'JCM_CollarUp_55': {
        'joint_base': 'Scapula',
        'rotation': (0, -55, 0),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ShldrDown_40': {
        'joint_base': 'Shoulder',
        'rotation': (0, 40, 0),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ShldrFwd_110': {
        'joint_base': 'Shoulder',
        'rotation': (0, 0, 110),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ShldrUp_90': {
        'joint_base': 'Shoulder',
        'rotation': (0, -90, 0),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ForeArmFwd_135': {
        'joint_base': 'Elbow',
        'rotation': (0, 0, 135),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ForeArmFwd_75': {
        'joint_base': 'Elbow',
        'rotation': (0, 0, 135),
        'keys': [(0.0, 0.0), (75.0/135.0, 1.0), (1.0, 1.0)],
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ThighBack_35': {
        'joint_base': 'Hip',
        'rotation': (0, 0, -35),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ThighFwd_57': {
        'joint_base': 'Hip',
        'rotation': (0, 0, 135),
        'keys': [(0.0, 0.0), (57.0/135.0, 1.0), (1.0, 1.0)],
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ThighFwd_115': {
        'joint_base': 'Hip',
        'rotation': (0, 0, 135),
        'keys': [(0.0, 0.0), (115.0/135.0, 1.0), (1.0, 1.0)],
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ThighSide_85': {
        'joint_base': 'Hip',
        'rotation': (0, -85, 0),
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ShinBend_155': {
        'joint_base': 'Knee',
        'rotation': (0, 0, -155),
        'keys': [(0.0, 0.0), (90.0/155.0, 0.0), (1.0, 1.0)],
        'mirror': True,
        'invert_axis': True
    },
    'JCM_ShinBend_90': {
        'joint_base': 'Knee',
        'rotation': (0, 0, -155),
        'keys': [(0.0, 0.0), (90.0/155.0, 1.0), (1.0, 1.0)],
        'mirror': True,
        'invert_axis': True
    }
}

TARGET_MAP = {
    'Jaw': 'JawEnd',
    'Neck': 'Head',
    'Scapula': 'Shoulder',
    'Shoulder': 'Elbow',
    'Elbow': 'Wrist',
    'Hip': 'Knee',
    'Knee': 'Ankle'
}


def createWtDrivers(node_name,
                    morph_name,
                    joint_base,
                    rotation,
                    radius=None,
                    keys=((0.0, 0.0), (1.0, 1.0)),
                    mirror=False,
                    invert_axis=False):
    bs_node = pm.ls(node_name)[0]

    radius = radius or sum(rot**2 for rot in rotation)**(1.0/2.0)

    if mirror:
        suffixes = ['_L', '_R']
    else:
        suffixes = ['_M']

    for suf in suffixes:
        m_suf = suf if mirror else ''
        if bs_node.hasAttr(morph_name+m_suf):
            morph = bs_node.attr(morph_name + m_suf)
        else:
            continue

        joint = pm.ls(joint_base + suf)[0]
        target = pm.ls(TARGET_MAP[joint_base]+suf)[0]

        if suf == '_R':
            inv = not invert_axis
        else:
            inv = invert_axis

        wd_node = pm.createNode(
            'weightDriver')
        pm.rename(wd_node.getParent(), 'wtDrv_' + morph.getAlias())

        pm.delete(pm.parentConstraint(joint, wd_node.getParent()))

        pm.rotate(wd_node, rotation, os=True, r=True)

        pm.parentConstraint(joint.getParent(),
                            wd_node.getParent(), mo=True)

        # Connect matrices
        wd_node.getParent().worldMatrix[0].connect(wd_node.readerMatrix)
        target.worldMatrix[0].connect(wd_node.driverMatrix)

        # Set weightDriver attributes
        wd_node.angle.set(radius)
        wd_node.invert.set(inv)
        wd_node.blendCurve[0].blendCurve_Interp.set(1)

        # Create remapValue nodes and set keys
        rv_node = pm.createNode(
            'remapValue', name='drv_' + morph.getAlias())
        for i, key in enumerate(keys):
            rv_node.value[i].value_Position.set(key[0])
            rv_node.value[i].value_FloatValue.set(key[1])

        wd_node.outWeight.connect(rv_node.inputValue)
        rv_node.outValue.connect(morph)
