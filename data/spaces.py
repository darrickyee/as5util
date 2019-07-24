

def getSpaceSwitchArgs(ctrl_name):

    ctrl_str = ctrl_name.split('_')
    ctrl = ctrl_str[0]
    side = '_' + ctrl_str[1]

    switch_args = {
        'drivenNode': ctrl_name[:2]+'Extra'+ctrl_name[2:],
        'controller': ctrl_name,
        'constraintType': 'parent',
        'driverSpaces': [{'DeformationSystem': 'world'},
                        {'Chest_M': 'chest'},
                        {'Scapula_L': 'clavicle'}],
        'spacesGrp': 'DeformationSystem',
        'attrName': 'spaces'
    }