DRIVER_LIST = [
    {
        'PoseName': 'NeckBack',
        'RefJntName': 'Neck',
        'DrvJntName': 'Head',
        'PoseRot': (0, 0, 17.5),
        'PoseRadius': 17.5,
        'Keys': {
            'JCM_NeckBack_27': [(0.0, 0.0), (13.5/17.5, 1.0), (1.0, 1.0)]
        },
        'Mirror':False,
        'InvertAxis':False
    },
    {
        'PoseName': 'NeckFwd',
        'RefJntName': 'Neck',
        'DrvJntName': 'Head',
        'PoseRot': (0, 0, -22.5),
        'PoseRadius': 22.5,
        'Keys': {
            'JCM_NeckFwd_35': [(0.0, 0.0), (17.5/22.5, 1.0), (1.0, 1.0)]
        },
        'Mirror':False,
        'InvertAxis':False
    },
    {
        'PoseName': 'ClavicleUp',
        'RefJntName': 'Clavicle',
        'DrvJntName': 'Shoulder',
        'PoseRot': (0, 55, 0),
        'PoseRadius': 55,
        'Keys': {
            'JCM_ClavicleUp_55': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'ShoulderDown',
        'RefJntName': 'Shoulder',
        'DrvJntName': 'Elbow',
        'PoseRot': (0, -40, 0),
        'PoseRadius': 40,
        'Keys': {
            'JCM_ShoulderDown_40': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'ShoulderFwd',
        'RefJntName': 'Shoulder',
        'DrvJntName': 'Elbow',
        'PoseRot': (0, 0, -110),
        'PoseRadius': 110,
        'Keys': {
            'JCM_ShoulderFwd_110': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'ShoulderUp',
        'RefJntName': 'Shoulder',
        'DrvJntName': 'Elbow',
        'PoseRot': (0, 90, 0),
        'PoseRadius': 90,
        'Keys': {
            'JCM_ShoulderUp_90': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'ElbowFwd',
        'RefJntName': 'Elbow',
        'DrvJntName': 'Wrist',
        'PoseRot': (0, 0, -135),
        'PoseRadius': 135,
        'Keys': {
            'JCM_ElbowFwd_135': [(0.0, 0.0), (1.0, 1.0)],
            'JCM_ElbowFwd_75':[(0.0, 0.0), (75.0/135.0, 1.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'HipBack',
        'RefJntName': 'Hip',
        'DrvJntName': 'Knee',
        'PoseRot': (0, 0, -35),
        'PoseRadius': 35,
        'Keys': {
            'JCM_HipBack_35': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'HipFwd',
        'RefJntName': 'Hip',
        'DrvJntName': 'Knee',
        'PoseRot': (0, 0, -135),
        'PoseRadius': 135,
        'Keys': {
            'JCM_HipFwd_115': [(0.0, 0.0), (115.0/135.0, 1.0), (1.0, 1.0)],
            'JCM_HipFwd_57':[(0.0, 0.0), (57.0/135.0, 1.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    # {
    #     'PoseName':'HipGroin',
    #     'RefJntName':'Hip',
    #     'DrvJntName':'HipPart1',
    #     'PoseRot':(0, -60, 45),
    #     'PoseRadius':60,
    #     'Keys':{
    #         'JCM_HipGroin':[(0.0, 0.0), (0.75, 1.0), (1.0, 1.0)]
    #         },
    #     'Mirror':True,
    #     'InvertAxis':True
    #     },
    {
        'PoseName': 'HipSide',
        'RefJntName': 'Hip',
        'DrvJntName': 'Knee',
        'PoseRot': (0, 85, 0),
        'PoseRadius': 85,
        'Keys': {
            'JCM_HipSide_85': [(0.0, 0.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
    {
        'PoseName': 'KneeBend',
        'RefJntName': 'Knee',
        'DrvJntName': 'Ankle',
        'PoseRot': (0, 0, 155),
        'PoseRadius': 155,
        'Keys': {
            'JCM_KneeBend_155': [(0.0, 0.0), (90.0/155.0, 0.0), (1.0, 1.0)],
            'JCM_KneeCompress':[(0.0, 0.0), (135.0/155.0, 0.0), (1.0, 1.0)],
            'JCM_KneeBend_90':[(0.0, 0.0), (90.0/155.0, 1.0), (1.0, 1.0)]
        },
        'Mirror':True,
        'InvertAxis':True
    },
]

md = {'JCM_ElbowFwd_135': {
    'Driver': 'Elbow',
    'Target': 'Wrist',
    'Rotation': (0, 0, -135),
    'Radius': 135,
    'Keys': [(0.0, 0.0), (1.0, 1.0)],
    'Mirror': True,
    'InvertAxis': True
},
    'JCM_ShoulderDown_40': None,
    'JCM_ShoulderFwd_110': None,
    'JCM_NeckBack_27': None,
    'JCM_KneeBend_90': None,
    'JCM_HipFwd_115': None,
    'JCM_HipFwd_57': None,
    'JCM_ClavicleUp_55': None,
    'JCM_ElbowFwd_75': None,
    'JCM_NeckFwd_35': None,
    'JCM_KneeBend_155': None,
    'JCM_KneeCompress': None,
    'JCM_ShoulderUp_90': None,
    'JCM_HipBack_35': None,
    'JCM_HipSide_85': None,
    'JCM_JawOpen_25': None}
