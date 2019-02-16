from functools import partial
import pymel.core as pm
from etc.shapes import ControlShapes

CONTROL_COLORS = {
    'Left': (1.0, 0.0, 0.0),
    'Right': (0.0, 0.0, 1.0),
    'Center': (0.15, 1.0, 1.0),
    'Other': (1.0, 1.0, 0.15)
}


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

    locs = [xform.getTranslation(space='world') for xform in [start, mid, end]]
    vec_basen = (locs[2] - locs[0]).normal()
    vec_mid = (locs[1] - locs[0])
    pole_vec = (vec_mid - vec_mid.dot(vec_basen)*vec_basen).normal()

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



def addSpace(target, control_obj, orient_only=False):
    """
    Adds a target space to the current control object.

    Parameters
    ----------
    target : pymel.nodetypes.Transform
        Target object for constraint
    control_obj : pymel.nodetypes.Transform
        Control object.  Target object will be added as constraint target for the parent "offset" transform for the control.
    orient_only : bool
        Use orient constraint? (the default is False, which uses a parent constraint)

    """

    if orient_only:
        constrain = pm.orientConstraint
        constraint_desc = 'orient'
    else:
        constrain = pm.parentConstraint
        constraint_desc = 'parent'

    # Add type checking?
    target_name = target.nodeName()
    control_offset = control_obj.getParent()

    constraint_obj = constrain(target, control_offset, mo=True)
    target_index = constraint_obj.getTargetList().index(target)
    constraint_wt_attr = constraint_obj.getWeightAliasList()[target_index]

    control_offset.addAttr("wt_{0}_{1}".format(
        constraint_desc, target_name), at=float, min=0.0, max=1.0, keyable=True)
    control_wt_attr = control_offset.attr(
        "wt_{0}_{1}".format(constraint_desc, target_name))

    pm.connectAttr(control_wt_attr, constraint_wt_attr)

    return(control_wt_attr)


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

    shape_args = ControlShapes.get(ctrl_type.lower(), ControlShapes['other'])

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


def createMayaNode(name, node_func=partial(pm.createNode, 'transform'), transform=None):
    new_node = node_func(name=name)
    if transform is not None:
        new_node.setTransformation(transform.getMatrix(ws=True))

    return new_node


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


class RigModule(object):
    def __init__(self, name, joints, rig_units=None, constraint_func=pm.orientConstraint):
        self._name = name
        self._joints = joints
        self._rig_units = rig_units
        self.constrain = constraint_func

        self._constraints = list()

    def build(self):

        # Build control groups
        for unit in self._rig_units:
            unit.build()

        # Transpose driver lists
        driver_list = zip(
            *[unit.drivers for unit in self._rig_units])

        # Constrain objects
        for i, j in enumerate(self._joints):
            self._constraints.append(
                [self.constrain(tgt, j) for tgt in driver_list[i]][0])

        self.postProcess()

    def postProcess(self):
        pass
