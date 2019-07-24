from abc import ABCMeta, abstractproperty
import pymel.core as pm


class AbstractJointMap(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractproperty
    def joint_map(self):
        pass

    @abstractproperty
    def custom_locs(self):
        pass

    @property
    def jnames(self):
        return list(self.joint_map.keys())

    @property
    def jnames_mapped(self):
        return [jname for jname in self.jnames if self.joint_map[jname]]

    @property
    def jnames_unmapped(self):
        return [jname for jname in self.jnames if not self.joint_map[jname]]

    def getJoints(self):
        if self.joint_map:
            return {jname: pm.ls(jname[:-2])[0] for jname in self.joint_map}

        return {}
