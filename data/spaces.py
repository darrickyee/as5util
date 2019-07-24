
_SPACE_LIST = [
    {'controller': 'IKArm',
     'driverSpaces': [
         {'DeformationSystem': 'World'},
         {'Chest_M': 'Chest'},
         {'Hip': 'Hip'},
         {'Knee': 'Knee'},
         {'Ankle': 'Ankle'}
     ]},
    {'controller': 'IKLeg',
     'driverSpaces': [
         {'DeformationSystem': 'World'},
         {'Pelvis_M': 'Pelvis'}
     ]},
    {'controller': 'FKHead_M',
     'driverSpaces': [
         {'DeformationSystem': 'World'},
         {'Neck_M': 'Neck'}
     ],
        'constraintType': 'orient'
     },
    {'controller': 'AimEye_M',
     'driverSpaces': [
         {'DeformationSystem': 'World'},
         {'Head_M': 'Head'}
     ]}
]

SPACE_ARGS = getSpaceSwitchArgs(_SPACE_LIST)


def getSpaceSwitchArgs(args_list):

    out_args = list()

    for arg_dict in args_list:
        if not arg_dict['controller'].endswith(('_L', '_M', '_R')):
            out_args.extend(
                [makeArgs(**side_args)
                 for side_args in symmetrizeArgs(arg_dict)]
            )
        else:
            out_args.append(makeArgs(**arg_dict))

    return out_args


def symmetrizeArgs(arg_dict):
    new_dict = arg_dict.copy()
    for side in '_L', '_R':
        new_dict['controller'] = arg_dict['controller'] + side
        new_dict['driverSpaces'] = [
            {(key+side if not key.endswith(('_M', 'DeformationSystem')) else key): val
             for key, val in drv_dict.items()}
            for drv_dict in arg_dict['driverSpaces']
        ]
        yield new_dict


def makeArgs(controller, **kwargs):

    base_dict = {
        'constraintType': 'parent',
        'spacesGrp': 'DeformationSystem',
        'attrName': 'Spaces',
        'driverSpaces': [{'DeformationSystem': 'World'},
                         {'Root_M': 'Root'}]
    }

    kwargs = {key: val for key, val in kwargs.items()
              if key in base_dict}

    ctrl = controller
    prefix = ''

    for pre in 'FK', 'IK', 'Aim':
        if controller.startswith(pre):
            ctrl = ctrl.lstrip(pre)
            prefix = pre

    kwargs['controller'] = controller
    kwargs['drivenNode'] = prefix+'Extra'+ctrl

    base_dict.update(kwargs)

    return base_dict
