DRIVER_LIST = [
	{
        'PoseName':'JawOpen',     # Root name for driver nodes
        'RefJntName':'Jaw',         # Name of reference location joint (without _L, _R, _M suffix)
        'DrvJntName':'JawEnd',    # Name of driver location joint (without _L, _R, _M suffix)
        'PoseRot':(0, 0, -25),      # Pose rotation relative to reference
        'PoseRadius':25,            # Weight driver radius (in degrees, for left-side joint)
        'Keys':{                    # Blendshape targets and (position, value) keys
            'JCM_JawOpen_25':[(0.0, 0.0), (0.5, 1.0), (1.0, 1.0)]
            },
        'Mirror':False,              # Create _L and _R drivers? (False: _M driver)
		'InvertAxis':False			# Invert pose reader axis (left side)?
        },
	{
        'PoseName':'NeckBack',
        'RefJntName':'Neck',
        'DrvJntName':'Head',
        'PoseRot':(0, 0, 17.5),
        'PoseRadius':17.5,
        'Keys':{
            'JCM_NeckBack_27':[(0.0, 0.0), (13.5/17.5, 1.0), (1.0, 1.0)]
            },
        'Mirror':False,
		'InvertAxis':False
        },
	{
        'PoseName':'NeckFwd',
        'RefJntName':'Neck',
        'DrvJntName':'Head',
        'PoseRot':(0, 0, -22.5),
        'PoseRadius':22.5,
        'Keys':{
            'JCM_NeckFwd_35':[(0.0, 0.0), (17.5/22.5, 1.0), (1.0, 1.0)]
            },
        'Mirror':False,
		'InvertAxis':False
        },
	{
        'PoseName':'ClavicleUp',
        'RefJntName':'Clavicle',
        'DrvJntName':'Shoulder',
        'PoseRot':(0, 55, 0),
        'PoseRadius':55,
        'Keys':{
            'JCM_ClavicleUp_55':[(0.0, 0.0), (1.0, 1.0)]
            },
        'Mirror':True,
		'InvertAxis':True
        },
	{
        'PoseName':'ShoulderFwd',
        'RefJntName':'Shoulder',
        'DrvJntName':'Elbow',
        'PoseRot':(0, 0, -110),
        'PoseRadius':110,
        'Keys':{
            'JCM_ShoulderFwd_110':[(0.0, 0.0), (1.0, 1.0)]
            },
        'Mirror':True,
		'InvertAxis':True
        },
	{
        'PoseName':'ShoulderUp',
        'RefJntName':'Shoulder',
        'DrvJntName':'Elbow',
        'PoseRot':(0, 90, 0),
        'PoseRadius':90,
        'Keys':{
            'JCM_ShoulderUp_90':[(0.0, 0.0), (1.0, 1.0)]
            },
        'Mirror':True,
		'InvertAxis':True
        }
]

print(dict.fromkeys([k for driver in DRIVER_LIST for k in driver['Keys']]))