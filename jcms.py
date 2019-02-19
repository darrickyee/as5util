import pymel.core as pm

TARGET_MAP = {
    'Jaw': 'JawEnd',
    'Neck': 'Head',
    'Clavicle': 'Shoulder',
    'Shoulder': 'Elbow',
    'Elbow': 'Wrist',
    'Hip': 'Knee',
    'Knee': 'Ankle'
}

MORPHS = ['JCM_JawOpen_25',
          'JCM_NeckBack_27',
          'JCM_NeckFwd_35',
          'JCM_ClavicleUp_55',
          'JCM_ShoulderDown_40',
          'JCM_ShoulderFwd_110',
          'JCM_ShoulderUp_90',
          'JCM_ElbowFwd_135',
          'JCM_ElbowFwd_75',
          'JCM_HipFwd_115',
          'JCM_HipFwd_57',
          'JCM_HipBack_35',
          'JCM_HipSide_85',
          'JCM_KneeBend_155',
          'JCM_KneeBend_90',
          'JCM_KneeCompress'
          ]

DRIVERS = {
    'JCM_JawOpen_25': {
        'joint': 'Jaw',
        'rotation': (0, 0, -25),
        'keys': [(0.0, 0.0), (0.5, 1.0), (1.0, 1.0)],
    },
    'JCM_ClavicleUp_55': {
        'joint': 'Clavicle',
        'rotation': (0, 55, 0),
        'mirror': True,
        'invert_axis': True
    }
}


class WtDrvBuilder(object):

    def __init__(self,
                 morph,
                 joint_name,
                 rotation,
                 radius=None,
                 keys=[(0.0, 0.0), (1.0, 1.0)],
                 mirror=False,
                 invert_axis=False):

        self.morph = morph
        self.morph_name = "{0}_{1}".format(morph.node(), morph.getAlias())
        self.joint_name = joint_name
        self.rotation = rotation
        self.radius = radius or max(abs(t) for t in rotation)
        self.keys = keys
        self.mirror = mirror
        self.invert_axis = invert_axis

    def buildDriver(self):

        if self.mirror:
            suf = ['_L', '_R']
        else:
            suf = ['_M']

        for s in suf:
            joint = pm.ls(self.joint_name + suf)[0]
            target = pm.ls(TARGET_MAP[self.joint_name + suf])[0]

            if s == '_R':
                inv = not self.invert_axis
            else:
                inv = self.invert_axis

            wd_node = pm.createNode(
                'weightDriver', name='wtDrv_' + str(self.morph))

            pm.delete(pm.parentConstraint(joint, wd_node.getParent()))

            pm.rotate(wd_node, self.rotation, os=True, r=True)

            pm.parentConstraint(joint.getParent(),
                                wd_node.getParent(), mo=True)

            # Connect matrices
            wd_node.getParent().worldMatrix[0].connect(wd_node.readerMatrix)
            target.worldMatrix[0].connect(wd_node.driverMatrix)

            # Set weightDriver attributes
            wd_node.angle.set(self.radius)
            wd_node.invert.set(self.invert_axis)
            wd_node.blendCurve[0].blendCurve_Interp.set(1)

            # Create remapValue nodes and set keys
            rv_node = pm.createNode(
                'remapValue', name='drv_' + str(self.morph))
            for i, key in enumerate(self.keys):
                rv_node.value[i].value_Position.set(key[0])
                rv_node.value[i].value_FloatValue.set(key[1])

            wd_node.outWeight.connect(rv_node.inputValue)
            rv_node.outValue.connect(morph)
