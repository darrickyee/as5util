from functools import partial
import pymel.core as pm
from .data import CTRL_SHAPES

CONTROL_COLORS = {
    'Left': (1.0, 0.0, 0.0),
    'Right': (0.0, 0.0, 1.0),
    'Center': (0.15, 1.0, 1.0),
    'Other': (1.0, 1.0, 0.15)
}


def lockAndHideAttrs(node_list, attr_list=('scaleX', 'scaleY', 'scaleZ')):
    for node in node_list:
        for attr_name in attr_list:
            node.setAttr(attr_name, lock=True, keyable=False)


def makeLockXYZ(xf_name, unlock=False):

    def lockFunc(xforms):
        xforms = pm.ls(xforms)
        xf_attrs = [xf_name+axis for axis in ('X', 'Y', 'Z')]

        for xform in xforms:
            for xf_attr in xf_attrs:
                xform.attr(xf_attr).set(k=unlock, lock=not unlock)

    return lockFunc


lockTranslate = makeLockXYZ('translate')
unlockTranslate = makeLockXYZ('translate', unlock=True)
lockRotate = makeLockXYZ('rotate')
unlockRotate = makeLockXYZ('rotate', unlock=True)
lockScale = makeLockXYZ('scale')
unlockScale = makeLockXYZ('scale', unlock=True)


def getPoleVector(start, mid, end):
    """
    Returns a unit pole vector for `start`, `mid`, and `end` transforms.
    The pole vector is (parallel to) the vector orthogonal to the vector
    between `start` and `end` that passes through `mid`.
    (Note that `start` and `end` are interchangeable.)

    Parameters
    ----------
    start : pm.nodetypes.Transform

    mid : pm.nodetypes.Transform

    end : pm.nodetypes.Transform


    Returns
    -------
    pm.datatypes.Vector

    """

    locs = [xform.getTranslation(space='world')
            for xform in [start, mid, end]]
    vec_basen = (locs[2] - locs[0]).normal()
    vec_mid = (locs[1] - locs[0])
    pole_vec = (vec_mid - vec_mid.dot(vec_basen)*vec_basen)

    return pole_vec


def orientJoint(joint, target, up_vector=(0, 1, 0), world_up=(0, 1, 0)):
    parented = False

    if target.getParent() == joint:
        parented = True
        target.setParent(None)

    pm.delete(pm.aimConstraint(target, joint,
                               upVector=up_vector, worldUpVector=world_up))
    pm.makeIdentity(joint, apply=True)

    if parented:
        target.setParent(joint)


def createControlCurve(name=None, ctrl_type='FK', size=1.0, color=(1.0, 1.0, 0.15)):
    """
    Creates a curve using predefined parameters.

    Parameters
    ----------
    name : str
        Desired curve name in Maya
    ctrlType : str
        Shape type, as defined in rig.ControlShapes (the default is 'FK')
    size : float
        Curve radius, in Maya scene units (the default is 1.0)
    color : tuple
        Tuple of RGB values in 0.0 to 1.0 range.  Set to None to use Maya's default color.

    """

    shape_args = CTRL_SHAPES.get(ctrl_type.lower(), CTRL_SHAPES['other'])

    if name:
        shape_args.update({'name': name})
    crv = pm.curve(**shape_args)

    if color:
        crv.setAttr('overrideColorRGB', color)
        crv.setAttr('overrideEnabled', True)
        crv.setAttr('overrideRGBColors', True)

    if size:
        crv.setScale([size, size, size])
        pm.makeIdentity(crv, apply=True)

    return(crv)


def createNodeChain(input_xforms, node_func=partial(pm.createNode, 'transform'), name_list=None, prefix='_'):
    if not isinstance(input_xforms, list):
        input_xforms = [input_xforms]

    if not name_list:
        name_list = [prefix + node.nodeName() for node in input_xforms]

    node_list = list()
    for node, node_name in zip(input_xforms, name_list):
        new_node = node_func(name=node_name)
        pm.delete(pm.parentConstraint(node, new_node))
        if node_list:
            new_node.setParent(node_list[-1])
        node_list.append(new_node)

    return node_list


def alignToWorldVector(xform, aim_x=(1, 0, 0), aim_y=(0, 1, 0), freeze=False):
    """
    Rotates transform so that axes align with specified world-space directions.

    Parameters
    ----------
    xform : pm.nt.Transform
        Transform to rotate.
    aim_x : tuple, optional
        World-space direction for the x-axis of `xform`.
    aim_y : tuple, optional
        World-space direction for the y-axis of `xform`.  If not orthogonal to `aim_x`,
        the y-axis will attempt to be as close as possible to this vector.
    freeze : bool, optional
        Freeze transformation if True. Default is False.

    """

    xform = pm.ls(xform)[0]

    xf_node = pm.createNode('transform')
    pm.move(xf_node, xform.getTranslation(ws=True))

    aim_node = pm.createNode('transform')
    pm.move(aim_node, xform.getTranslation(space='world') + aim_x)

    pm.delete(pm.aimConstraint(aim_node, xf_node,
                               worldUpVector=aim_y), aim_node)

    xform.setRotation(xf_node.getRotation(ws=True), ws=True)
    pm.delete(xf_node)

    if freeze:
        pm.makeIdentity(xform, apply=True)
