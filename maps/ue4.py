from .abstract import AbstractJointMap


class Ue4Map(AbstractJointMap):

    @property
    def joint_map(self):
        return {'Root_M': 'pelvis',
                'Spine1_M': 'spine_01',
                'Spine2_M': 'spine_02',
                'Chest_M': 'spine_03',
                'Neck_M': 'neck_01',
                'Head_M': 'head',
                'HeadEnd_M': '',
                'Hip_R': 'thigh_r',
                'Knee_R': 'calf_r',
                'Ankle_R': 'foot_r',
                'FootSideInner_R': '',
                'FootSideOuter_R': '',
                'Heel_R': '',
                'Toes_R': 'ball_r',
                'ToesEnd_R': '',
                'Scapula_R': 'clavicle_r',
                'Shoulder_R': 'upperarm_r',
                'Elbow_R': 'lowerarm_r',
                'Wrist_R': 'hand_r',
                'ThumbFinger1_R': 'thumb_01_r',
                'ThumbFinger2_R': 'thumb_02_r',
                'ThumbFinger3_R': 'thumb_03_r',
                'ThumbFinger4_R': '',
                'IndexFinger1_R': 'index_01_r',
                'IndexFinger2_R': 'index_02_r',
                'IndexFinger3_R': 'index_03_r',
                'IndexFinger4_R': '',
                'MiddleFinger1_R': 'middle_01_r',
                'MiddleFinger2_R': 'middle_02_r',
                'MiddleFinger3_R': 'middle_03_r',
                'MiddleFinger4_R': '',
                'RingFinger1_R': 'ring_01_r',
                'RingFinger2_R': 'ring_02_r',
                'RingFinger3_R': 'ring_03_r',
                'RingFinger4_R': '',
                'PinkyFinger1_R': 'pinky_01_r',
                'PinkyFinger2_R': 'pinky_02_r',
                'PinkyFinger3_R': 'pinky_03_r',
                'PinkyFinger4_R': ''
                }

    @property
    def custom_xforms(self):
        return {
            'ThumbFinger4_R': (0.2874, -2.7302, 1.9647),
            'HeadEnd_M': (0.0, 16.947, -0.0),
            'FootSideOuter_R': (-5.9033, -2.8476, -1.459),
            'MiddleFinger4_R': (-1.1072, -2.7383, 0.7222),
            'RingFinger4_R': (-0.7164, -2.6562, 0.4484),
            'FootSideInner_R': (5.2681, -2.8476, 1.6574),
            'Ankle_R': (-3.2561, -39.6016, -6.3413),
            'ToesEnd_R': (-0.8055, -2.8476, 6.1033),
            'IndexFinger4_R': (-0.574, -2.6662, 0.7217),
            'PinkyFinger4_R': (-0.8898, -2.2295, 0.1648)
        }

    @property
    def control_names(self):
        return ['FKMiddleFinger3_R',
                'FKThumbFinger1_R',
                'FKMiddleFinger2_R',
                'FKMiddleFinger1_R',
                'FKToes_L',
                'RollHeel_R',
                'IKLeg_R',
                'FKToes_R',
                'FKKnee_L',
                'FKHip_L',
                'FKAnkle_L',
                'FKKnee_R',
                'FKAnkle_R',
                'FKIKSpine_M',
                'FKIKArm_R',
                'FKIKLeg_R',
                'RootX_M',
                'FKIKArm_L',
                'FKIKLeg_L',
                'FKElbow_L',
                'FKShoulder_L',
                'FKRingFinger3_L',
                'FKRingFinger2_L',
                'FKRingFinger1_L',
                'HipSwinger_M',
                'FKChest_M',
                'FKSpine2_M',
                'FKSpine1_M',
                'RollToes_L',
                'RollToesEnd_L',
                'RollHeel_L',
                'IKLeg_L',
                'IKArm_L',
                'PoleLeg_L',
                'IKToes_L',
                'PoleArm_L',
                'FKPinkyFinger3_L',
                'FKPinkyFinger2_L',
                'FKPinkyFinger1_L',
                'FKIndexFinger3_L',
                'FKIndexFinger2_L',
                'FKIndexFinger1_L',
                'FKThumbFinger3_L',
                'FKRoot_M',
                'IKScapula_R',
                'IKScapula_L',
                'FKHead_M',
                'FKWrist_L',
                'Main',
                'FKNeck_M',
                'FKWrist_R',
                'FKElbow_R',
                'FKThumbFinger2_R',
                'FKThumbFinger3_R',
                'IKArm_R',
                'PoleLeg_R',
                'IKToes_R',
                'RollToes_R',
                'RollToesEnd_R',
                'FKHip_R',
                'FKRingFinger2_R',
                'FKRingFinger3_R',
                'FKShoulder_R',
                'FKPinkyFinger1_R',
                'FKPinkyFinger3_R',
                'FKRingFinger1_R',
                'FKPinkyFinger2_R',
                'FKThumbFinger2_L',
                'FKThumbFinger1_L',
                'FKMiddleFinger3_L',
                'FKMiddleFinger2_L',
                'FKMiddleFinger1_L',
                'Fingers_R',
                'Fingers_L',
                'PoleArm_R',
                'IKSpine1_M',
                'IKSpine3_M',
                'IKSpine2_M',
                'FKIndexFinger2_R',
                'FKIndexFinger3_R',
                'FKIndexFinger1_R']