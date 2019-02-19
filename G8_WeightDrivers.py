import pymel.core as pm

Head = True

if Head:
    # from G8_DriverList_Head import DRIVER_LIST
    wdSuf = 'Head'
else:
    # from G8_DriverList_Body import DRIVER_LIST
    wdSuf = 'Body'

BlendShapeNodeName = 'Morphs' + wdSuf

# Delete existing drivers
wdlst = pm.ls('wtDrv' + wdSuf + '*', type='weightDriver')
if wdlst:
    for i in wdlst:
        t = i.getParent()
        DeleteList = [t]
        DeleteList = DeleteList + \
            list(set(pm.listConnections(
                t, type=['parentConstraint', 'aimConstraint'], exactType=True)))
        DeleteList = DeleteList + \
            list(set(pm.listConnections(
                i, type=['remapValue'], exactType=True)))
        pm.delete(DeleteList)


def GetNodeByName(name):
    if (len(pm.ls(name)) == 1):
        myNode = pm.ls(name)[0]
    return(myNode)


# Get blendshape node
bsNode = GetNodeByName(BlendShapeNodeName)

for d in DRIVER_LIST:

    # Suffix for node/attribute names
    if (d['Mirror'] == True):
        suf = ['_L', '_R']
    else:
        suf = ['_M']

    for s in suf:
        # Get joints
        refJnt = GetNodeByName(d['RefJntName'] + s)
        drvJnt = GetNodeByName(d['DrvJntName'] + s)
        # Reverse axis inversion for right side
        if s == '_R':
            inv = not d['InvertAxis']
        else:
            inv = d['InvertAxis']

        # Create weightDriver
        wd = pm.createNode('weightDriver', name='wtDrv' +
                           wdSuf + '_' + d['PoseName'] + s + 'Shape')
        print('Created weightDriver ' + wd)

        # Copy transforms
        wd.getParent().setMatrix(refJnt.getMatrix(ws=True), ws=True)
        print('Matched transformation ' + wd + ' to ' + refJnt)

        # Rotate weightDriver to target pose
        pm.rotate(wd, d['PoseRot'], os=True, r=True)

        # Constrain weightDriver transform to parent
        if d['RefJntName'] in ['Elbow', 'Knee']:
            aimJnt = refJnt.getParent()
            if s == '_R':
                aimVec = (1.0, 0.0, 0.0)
                upVec = (0.0, 1.0, 0.0)
            else:
                aimVec = (-1.0, 0.0, 0.0)
                upVec = (0.0, -1.0, 0.0)
            pm.aimConstraint(aimJnt, wd.getParent(), aim=aimVec, u=upVec,
                             mo=True, worldUpType='object', worldUpObject=drvJnt)
            pm.parentConstraint(refJnt.getParent(),
                                wd.getParent(), mo=True, sr=['x', 'y', 'z'])
        else:
            pm.parentConstraint(refJnt.getParent(), wd.getParent(), mo=True)
        # print('Constrained ' + wd + ' to parent of ' + refJnt)

        # Connect matrices
        wd.getParent().worldMatrix[0].connect(wd.readerMatrix)
        drvJnt.worldMatrix[0].connect(wd.driverMatrix)

        # Set weightDriver attributes
        wd.angle.set(d['PoseRadius'])
        wd.invert.set(inv)
        wd.blendCurve[0].blendCurve_Interp.set(1)

        # Create remapValue nodes & set keys
        for k in d['Keys'].keys():
            if s == '_M':
                ms = ''
            else:
                ms = s

            # Get target blendshape attr
            bsAttr = bsNode.attr(k + ms)

            # Create remapValue node
            vn = pm.createNode('remapValue', name='drv_' +
                               d['PoseName'] + '_' + k + ms)

            # Set keys
            for i in range(len(d['Keys'][k])):
                vn.value[i].value_Position.set(d['Keys'][k][i][0])
                vn.value[i].value_FloatValue.set(d['Keys'][k][i][1])

            # Connect blendshapes
            wd.outWeight.connect(vn.inputValue)
            vn.outValue.connect(bsAttr)
