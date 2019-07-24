# Embedded file name: D:\data\Core\tools\RTTools\spaceSwitchTool\spaceSwitchSetup.py
"""

@author: Riham Toulan
@copyright: 2017 (c) purplepuppet AB. All Rights Reserved
"""
__author__ = 'Riham Toulan'
__license__ = 'Private'
__copyright__ = '2017 (c) purplepuppet AB. %s License' % __license__
__date__ = '2017-08-25'
__version__ = '1.0'
import os
import maya.cmds as cmds
import maya.OpenMaya as OpenMaya
import maya.OpenMayaUI as mui
import spaceSwitchScripts as switchUtils
try:
    from PySide import QtGui
    from PySide.QtCore import *
    from PySide.QtGui import *
    QtWidgets = QMainWindow()
    from shiboken import wrapInstance
except ImportError:
    from PySide2 import QtGui, QtWidgets
    from PySide2.QtCore import *
    from PySide2.QtWidgets import *
    from shiboken2 import wrapInstance

from functools import partial
import logging
logging.basicConfig()
log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)

def getMayaWindow():
    ptr = mui.MQtUtil.mainWindow()
    if ptr:
        return wrapInstance(long(ptr), QMainWindow)


def show():
    global UiWindow
    try:
        UiWindow.close()
    except:
        pass

    UiWindow = SpaceSwitchWindow()
    UiWindow.show()
    return UiWindow


class SpaceSwitchWindow(QMainWindow):
    """
    classdocs
    """

    def __init__(self, parent = getMayaWindow()):
        super(SpaceSwitchWindow, self).__init__(parent)
        self.setWindowTitle('PP_SpaceSwitchTool %s. %s' % (__version__, __copyright__))
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.tabWidget = QTabWidget()
        self.mainLayout = QVBoxLayout()
        self.tabWidget.setLayout(self.mainLayout)
        self.setCentralWidget(self.tabWidget)
        self.tabWidget.setObjectName('centralwidget')
        self.createSpacesTab = QWidget()
        self.editSpacesTab = QWidget()
        self.bakeSpacesTab = QWidget()
        self.tabWidget.addTab(self.createSpacesTab, 'Create Spaces ')
        self.tabWidget.addTab(self.bakeSpacesTab, 'Bake Spaces ')
        self.tabWidget.addTab(self.editSpacesTab, 'Edit And Debug ')
        self.createSpacesTab.setObjectName('createSpaces_TAB')
        self.editSpacesTab.setObjectName('editSpaces_TAB')
        self.bakeSpacesTab.setObjectName('bakeSpaces_TAB')
        self.createSpacesTabVLO = QVBoxLayout(self.createSpacesTab)
        self.editSpacesTabVLO = QVBoxLayout()
        self.bakeSpacesTabVLO = QVBoxLayout()
        self.bakeSpacesTabVLO.setAlignment(Qt.AlignTop)
        self.bakeSpacesTab.setLayout(self.bakeSpacesTabVLO)
        self.editSpacesTab.setLayout(self.editSpacesTabVLO)
        self.switchControllerLabel = QLabel('&Target Node:')
        self.switchControllerEdit = QLineEdit()
        self.switchControllerLabel.setBuddy(self.switchControllerEdit)
        self.switchControllerButton = QPushButton('<<')
        self.switchControllerEdit.setToolTip('Add the node you want to drive with the spaces.')
        self.switchControllerLabel.setToolTip('The node you want to drive with the spaces.')
        self.switchControllerButton.setToolTip('Select the target node in the scene and press to add to the name field.')
        self.attributeNameLabel = QLabel('&Switch Attribute:')
        self.attributeNameEdit = QLineEdit()
        self.attributeNameLabel.setBuddy(self.attributeNameEdit)
        self.attributeNameEdit.setToolTip('The space switch attribute name. Only Enum attribute types are accepted..')
        self.attributeNameLabel.setToolTip('Add the space switch attribute name.')
        self.spacesLabel = QLabel('&Spaces Group:')
        self.spacesEdit = QLineEdit()
        self.spacesEdit.setObjectName('spacesGrp_LineEdit')
        self.spacesLabel.setBuddy(self.spacesEdit)
        self.spacesEdit.setToolTip('The spaces parent group if you have an existing one in the scene.')
        self.spacesLabel.setToolTip('The spaces parent group if you have an existing one in the scene.')
        self.spacesEdit.setPlaceholderText('Optional')
        self.spacesButton = QPushButton('<<')
        self.drivenNodeLabel = QLabel('&Parent Group:')
        self.drivenNodeEdit = QLineEdit()
        self.drivenNodeLabel.setBuddy(self.drivenNodeEdit)
        self.drivenNodeButton = QPushButton('<<')
        self.drivenNodeEdit.setToolTip('Add the node you want to drive with the spaces.')
        self.drivenNodeLabel.setToolTip('The node name you want to drive with the spaces.')
        self.drivenNodeButton.setToolTip('Select the node you want to drive with the spaces and press to add to the name field.')
        self.constraintLabel = QLabel('&Switch Constraint:')
        self.constraintComboBox = QComboBox()
        self.constraintLabel.setBuddy(self.constraintComboBox)
        self.constraintComboBox.addItems(['Parent Constraint', 'Point Constraint', 'Orient Constraint'])
        self.constraintView = QListView()
        self.constraintView.setSpacing(4)
        self.constraintComboBox.setView(self.constraintView)
        self.clearInfoButton = QPushButton('&Clear Info')
        self.infoGB = QGroupBox('Info')
        self.gridLayout = QGridLayout()
        self.infoGB.setLayout(self.gridLayout)
        self.gridLayout.addWidget(self.switchControllerLabel, 0, 0)
        self.gridLayout.addWidget(self.switchControllerEdit, 0, 1)
        self.gridLayout.addWidget(self.switchControllerButton, 0, 2)
        self.gridLayout.addWidget(self.attributeNameLabel, 2, 0)
        self.gridLayout.addWidget(self.attributeNameEdit, 2, 1)
        self.gridLayout.addWidget(self.drivenNodeLabel, 1, 0)
        self.gridLayout.addWidget(self.drivenNodeEdit, 1, 1)
        self.gridLayout.addWidget(self.drivenNodeButton, 1, 2)
        self.gridLayout.addWidget(self.constraintLabel, 3, 0)
        self.gridLayout.addWidget(self.constraintComboBox, 3, 1)
        self.gridLayout.addWidget(self.spacesLabel, 4, 0)
        self.gridLayout.addWidget(self.spacesEdit, 4, 1)
        self.gridLayout.addWidget(self.spacesButton, 4, 2)
        self.gridLayout.addWidget(self.clearInfoButton, 5, 0, 1, 3)
        for btn in [self.switchControllerButton, self.drivenNodeButton]:
            btn.setMinimumWidth(40)
            self.gridLayout.setAlignment(btn, Qt.AlignHCenter)

        self.createSpacesTabVLO.addWidget(self.infoGB)
        self.createParentButton = QPushButton('Create a Parent Space Switch Group')
        self.utilitiesGB = QGroupBox('Utilities')
        self.utilitiesGLO = QGridLayout()
        self.utilitiesGB.setLayout(self.utilitiesGLO)
        self.utilitiesGLO.addWidget(self.createParentButton, 0, 0)
        self.createSpacesTabVLO.addWidget(self.utilitiesGB)
        self.spacesGB = QGroupBox('Spaces')
        self.spacesGBVLO = QVBoxLayout()
        self.spacesGB.setLayout(self.spacesGBVLO)
        self.createSpacesTabVLO.addWidget(self.spacesGB)
        self.scrollArea = QScrollArea()
        self.scrollArea.setAcceptDrops(True)
        self.scrollArea.setObjectName('spaces_ScrollArea')
        self.scrollContent = DropFrame(self)
        self.scrollArea.setWidget(self.scrollContent)
        self.scrollArea.setWidgetResizable(True)
        self.addSpaceButton = QPushButton('Add A New Space')
        self.clearSpacesButton = QPushButton('&Clear Spaces')
        self.addSpaceButton.setObjectName('addSpace_BTN')
        self.spacesGBVLO.addWidget(self.addSpaceButton)
        self.spacesGBVLO.addWidget(self.scrollArea)
        self.spacesGBVLO.addWidget(self.clearSpacesButton)
        self._editFrames = []
        self.generateSpacesButton = QPushButton('Generate Spaces')
        self.generateSpacesButton.setObjectName('generateSpaces_BTN')
        self.spacesGBVLO.addWidget(self.generateSpacesButton)
        self.chooseSpaceGB = QGroupBox('Choose A Space')
        self.chooseSpaceGrid = QGridLayout()
        self.chooseSpaceGB.setLayout(self.chooseSpaceGrid)
        self.bakeSwitcherNodeLabel = QLabel('&Target Node:')
        self.bakeSwitcherNodeEdit = QLineEdit()
        self.bakeSwitcherNodeLabel.setBuddy(self.bakeSwitcherNodeEdit)
        self.bakeSwitcherNodeButton = QPushButton('<<')
        self.bakeSwitcherNodeButton.setMinimumWidth(40)
        self.bakeSwitcherNodeEdit.setToolTip('Add the target Node that has the space switch attribute on.')
        self.bakeSwitcherNodeLabel.setToolTip('The target Node you switch to different spaces.')
        self.bakeSwitcherNodeButton.setToolTip('Select the target Node in the scene and press to add to the name field.')
        self.availableSpacesLabel = QLabel('&Available Spaces:')
        self.availableSpacesComboBox = QComboBox()
        self.availableSpacesLabel.setBuddy(self.availableSpacesComboBox)
        self.availableSpacesView = QListView()
        self.availableSpacesView.setSpacing(2)
        self.availableSpacesComboBox.setView(self.availableSpacesView)
        self.availableSpacesComboBox.setToolTip('Choose the space you would like to bake.')
        self.availableSpacesLabel.setToolTip('The spaces on the switcher Node.')
        self.chooseSpaceGrid.addWidget(self.bakeSwitcherNodeLabel, 0, 0)
        self.chooseSpaceGrid.addWidget(self.bakeSwitcherNodeEdit, 0, 1)
        self.chooseSpaceGrid.addWidget(self.bakeSwitcherNodeButton, 0, 2)
        self.chooseSpaceGrid.addWidget(self.availableSpacesLabel, 1, 0)
        self.chooseSpaceGrid.addWidget(self.availableSpacesComboBox, 1, 1)
        self.bakingOptionsGB = QGroupBox('Baking Options')
        self.bakingOptionsVLO = QVBoxLayout()
        self.bakingOptionsGB.setLayout(self.bakingOptionsVLO)
        self.radioButtonHLO = QHBoxLayout()
        self.timelineRB = QRadioButton('Use Timeline Range')
        self.playbackRangeRB = QRadioButton('Use Playback Range')
        self.customRangeRB = QRadioButton('Custom Range')
        self.bakeFramesGrid = QGridLayout()
        self.bakeStartFrameLabel = QLabel('&Start Frame:')
        self.bakeStartFrameLabel.setObjectName('startFrame_Label')
        self.bakeStartFrameSpin = QSpinBox()
        self.bakeStartFrameLabel.setBuddy(self.bakeStartFrameSpin)
        self.bakeEndFrameLabel = QLabel('&End Frame:')
        self.bakeEndFrameLabel.setObjectName('endFrame_Label')
        self.bakeEndFrameSpin = QSpinBox()
        self.bakeEndFrameLabel.setBuddy(self.bakeEndFrameSpin)
        for spin in [self.bakeEndFrameSpin, self.bakeStartFrameSpin]:
            spin.setMinimumWidth(70)
            spin.setMinimum(-10000000)
            spin.setMaximum(10000000)
            spin.setValue(0)

        for spin in [self.bakeEndFrameLabel, self.bakeStartFrameLabel]:
            spin.setMinimumWidth(70)

        self.bakeSpaceButton = QPushButton('Bake Space')
        self.bakeSpaceButton.setObjectName('bakeSpace_BTN')
        self.bakeFramesGrid.addWidget(self.bakeStartFrameLabel, 0, 0)
        self.bakeFramesGrid.addWidget(self.bakeStartFrameSpin, 0, 1)
        self.bakeFramesGrid.addWidget(self.bakeEndFrameLabel, 1, 0)
        self.bakeFramesGrid.addWidget(self.bakeEndFrameSpin, 1, 1)
        self.bakeFramesGrid.addItem(QSpacerItem(10, 10, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 3)
        self.bakeFramesGrid.addItem(QSpacerItem(10, 10, QSizePolicy.Expanding, QSizePolicy.Minimum), 1, 3)
        for wid in [self.timelineRB, self.playbackRangeRB, self.customRangeRB]:
            self.radioButtonHLO.addWidget(wid)

        self.bakingOptionsVLO.addLayout(self.radioButtonHLO)
        self.bakingOptionsVLO.addLayout(self.bakeFramesGrid)
        self.bakeSpacesTabVLO.addWidget(self.chooseSpaceGB)
        self.bakeSpacesTabVLO.addWidget(self.bakingOptionsGB)
        self.bakeSpacesTabVLO.addWidget(self.bakeSpaceButton)
        self.bakeSpacesTabVLO.addStretch(0)
        self.radioButtonHLO.setContentsMargins(11, 0, 0, 11)
        self.editInfoGB = QGroupBox('Info')
        self.editInfoGBGLO = QGridLayout()
        self.editInfoGB.setLayout(self.editInfoGBGLO)
        self.editSpaceNodeLabel = QLabel('&Target Node:')
        self.editSpaceNodeEdit = QLineEdit()
        self.editSpaceNodeLabel.setBuddy(self.editSpaceNodeEdit)
        self.editSpaceNodeButton = QPushButton('<<')
        self.editSpaceNodeEdit.setToolTip('Add the target Node that has space switch attribute on.')
        self.editSpaceNodeLabel.setToolTip('The target Node you use switch between different spaces.')
        self.editSpaceNodeButton.setToolTip('Select the target Node in the scene and press to add to the name field.')
        self.editDrivenNodeLabel = QLabel('&Driven Node:')
        self.editDrivenNodeEdit = QLineEdit()
        self.editDrivenNodeLabel.setBuddy(self.editDrivenNodeEdit)
        self.editDrivenNodeButton = QPushButton('Select')
        self.editDrivenNodeEdit.setReadOnly(True)
        self.editDrivingConstLabel = QLabel('&Driving Constraint:')
        self.editDrivingConstEdit = QLineEdit()
        self.editDrivingConstLabel.setBuddy(self.editDrivingConstEdit)
        self.editDrivingConstButton = QPushButton('Select')
        self.editDrivingConstEdit.setReadOnly(True)
        self.editInfoGBGLO.addWidget(self.editSpaceNodeLabel, 0, 0)
        self.editInfoGBGLO.addWidget(self.editSpaceNodeEdit, 0, 1)
        self.editInfoGBGLO.addWidget(self.editSpaceNodeButton, 0, 2)
        self.editInfoGBGLO.addWidget(self.editDrivenNodeLabel, 1, 0)
        self.editInfoGBGLO.addWidget(self.editDrivenNodeEdit, 1, 1)
        self.editInfoGBGLO.addWidget(self.editDrivenNodeButton, 1, 2)
        self.editInfoGBGLO.addWidget(self.editDrivingConstLabel, 2, 0)
        self.editInfoGBGLO.addWidget(self.editDrivingConstEdit, 2, 1)
        self.editInfoGBGLO.addWidget(self.editDrivingConstButton, 2, 2)
        self.driversGB = QGroupBox('Drivers')
        self.driversGBVLO = QVBoxLayout()
        self.driversGB.setLayout(self.driversGBVLO)
        self.driversScrollArea = QScrollArea()
        self.driversScrollArea.setAcceptDrops(True)
        self.driversScrollArea.setObjectName('editDrivers_ScrollArea')
        self.driversContent = DropFrame(self)
        self.driversScrollArea.setWidget(self.driversContent)
        self.driversScrollArea.setWidgetResizable(True)
        self.driversGBVLO.addWidget(self.driversScrollArea)
        self.selectDriversButton = QPushButton('Select All Drivers')
        self.selectDriversButton.setObjectName('selectDrivers_BTN')
        self.driversGBVLO.addWidget(self.selectDriversButton)
        self.deleteSetupButton = QPushButton('Delete Spaces Setup!')
        self.deleteSetupButton.setObjectName('deleteSetup_BTN')
        for wid in [self.editInfoGB, self.driversGB, self.deleteSetupButton]:
            self.editSpacesTabVLO.addWidget(wid)

        self.styleSheet = '\n        QTabWidget{background-color: #292929;}\n        QTabBar:tab{background-color: #343434; font-size: 9pt; border: 2px solid #292929; border-bottom-color: #292929;border-top-left-radius: 1px;border-top-right-radius: 1px;min-width: 8ex;padding: 8px;}\n\n        QTabBar:tab:selected {\n            background-color: #292929;\n        }\n        QTabBar:tab:!selected {\n            margin-top: 2px;\n            color: #8b8b8b;\n        }\n        QGroupBox{background-color: #292929; border: 1px solid black; border-radius: 6px; margin-top: 6px;}\n        QGroupBox:title {subcontrol-origin: margin;left: 7px; padding: 0px 5px 0px 5px;}\n        QComboBox{background-color: #40546b; font-weight:bold;}\n        QLineEdit{background-color: #40546b;}\n        QLabel {color:#43bc99;}\n        QRadioButton {color:#43bc99; font-size: 9pt; font-weight:bold}\n        QPushButton{background-color:#393a3c; color:#43bc99;font-weight:bold; font-size: 9pt;}\n        #generateSpaces_BTN{font-size: 10pt; font-weight:bold; background-color:#387A7A; color:white;}\n        #bakeSpace_BTN{font-size: 10pt; font-weight:bold; background-color:#387A7A; color:white;}\n        #startFrame_Label{color:#bababa;}\n        #startFrame_Label:disabled{color:#424242;}\n        #endFrame_Label{color:#bababa;}\n        #endFrame_Label:disabled{color:#424242;}\n        '
        self.setStyleSheet(self.styleSheet)
        for gb in [self.gridLayout,
         self.spacesGBVLO,
         self.bakingOptionsVLO,
         self.chooseSpaceGrid,
         self.editInfoGBGLO,
         self.driversGBVLO]:
            gb.setContentsMargins(11, 15, 11, 11)

        self.switchControllerButton.clicked.connect(partial(getSelected, self.switchControllerEdit))
        self.drivenNodeButton.clicked.connect(partial(getSelected, self.drivenNodeEdit))
        self.spacesButton.clicked.connect(partial(getSelected, self.spacesEdit))
        self.switchControllerEdit.textChanged.connect(self.fillInfoFromField)
        self.clearInfoButton.clicked.connect(self.clearCreateSpacesInfo)
        self.createParentButton.clicked.connect(self.createParentGroup)
        self.addSpaceButton.clicked.connect(partial(self.addSpaceLayout, True))
        self.clearSpacesButton.clicked.connect(self.clearSpacesFrame)
        self.generateSpacesButton.clicked.connect(self.generateSpaces)
        self.bakeSwitcherNodeButton.clicked.connect(self.loadAvailableSpaces)
        self.timelineRB.toggled.connect(self.useTimeline)
        self.playbackRangeRB.toggled.connect(self.usePlayback)
        self.customRangeRB.toggled.connect(self.useCustomFrameRange)
        self.bakeSpaceButton.clicked.connect(self.bakeSpace)
        self.editSpaceNodeButton.clicked.connect(self.loadEditUI)
        self.editDrivingConstButton.clicked.connect(partial(self.selectFieldNodes, self.editDrivingConstEdit))
        self.editDrivenNodeButton.clicked.connect(partial(self.selectFieldNodes, self.editDrivenNodeEdit))
        self.selectDriversButton.clicked.connect(self.selectAllDrivers)
        self.deleteSetupButton.clicked.connect(self.deleteSetup)
        self.setBakeWidgetsState(False)
        self.driversContent.dropped.connect(self.editSpacesSetup)
        self.settingsFile = ''
        self._editSwitcherNode = None
        self._editConstraintNode = None
        self._editDrivenNode = None
        self._editFrames = []
        self.callback_list = []
        self.readSettings()
        self.loadRecentSettings()
        self.addCallBacks()
        return

    def addSpaceLayout(self, selectionBased = False):
        if selectionBased:
            sel = cmds.ls(sl=True)
            if not sel:
                newSpace = SpacesFrame(self)
                self.scrollContent.layout().addWidget(newSpace)
                self.scrollArea.setWidgetResizable(True)
                return newSpace
            for driver in sel:
                newSpace = SpacesFrame(self)
                newSpace.driverNodeEdit.setText(driver)
                newSpace.displayNameEdit.setText(driver)
                self.scrollContent.layout().addWidget(newSpace)

        else:
            newSpace = SpacesFrame(self)
            self.scrollContent.layout().addWidget(newSpace)
            self.scrollArea.setWidgetResizable(True)
            return newSpace

    @property
    def spacesLayouts(self):
        orderedSpaces = []
        spaceFrames = self.scrollContent.findChildren(QFrame, 'spaces_Frame')
        if not spaceFrames:
            return orderedSpaces
        for i in range(len(spaceFrames)):
            orderedSpaces.insert(self.scrollContent.layout().indexOf(spaceFrames[i]), spaceFrames[i])

        return orderedSpaces

    @property
    def drivenNode(self):
        return str(self.drivenNodeEdit.text())

    @property
    def constraintType(self):
        constraintType = self.constraintComboBox.currentIndex()
        if constraintType == 1:
            return 'point'
        if constraintType == 2:
            return 'orient'
        if constraintType == 0:
            return 'parent'

    @property
    def switchController(self):
        return str(self.switchControllerEdit.text())

    @property
    def switchAttribute(self):
        return str(self.attributeNameEdit.text())

    @property
    def spacesGroup(self):
        spacesGrp = str(self.spacesEdit.text())
        if not spacesGrp:
            return
        elif spacesGrp.lower() == 'optional':
            return
        else:
            return spacesGrp

    def getSpaceDriverNode(self, spaceFrame):
        return spaceFrame.driverNode

    def getSpaceDisplayName(self, spaceFrame):
        return spaceFrame.displayName

    def generateSpaces(self):
        cmds.undoInfo(openChunk=True)
        try:
            try:
                if not self.drivenNode:
                    self.missingValuesWarnings('driven node not Specified.', self.drivenNodeEdit)
                    return
                if not self.switchController:
                    self.missingValuesWarnings('switch controller not Specified.', self.switchControllerEdit)
                    return
                if not self.switchAttribute:
                    self.missingValuesWarnings('attribute name not Specified.', self.attributeNameEdit)
                    return
                for required in [self.drivenNode, self.switchController]:
                    if not cmds.objExists(required):
                        self.missingValuesWarnings("%s doesn't exist in the scene" % required)
                        return

                if not self.spacesLayouts:
                    self.missingValuesWarnings('No spaces given to add. Please add a space first.')
                    return
                if not self.spacesGroup or not cmds.objExists(self.spacesGroup):
                    log.warning("%s doesn't exist in the scene.Creating a default spaces group!" % self.spacesGroup)
                    cmds.select(cmds.group(em=1, n='spaces_GRP#', w=1))
                    getSelected(self.spacesEdit)
                if not attributeExists(self.spacesGroup, 'PP_spacesGroup'):
                    cmds.addAttr(self.spacesGroup, longName='PP_spacesGroup', at='message')
                if not attributeExists(self.switchController, 'PP_spacesGroup'):
                    cmds.addAttr(self.switchController, longName='PP_spacesGroup', at='message')
                try:
                    cmds.connectAttr('%s.PP_spacesGroup' % self.spacesGroup, '%s.PP_spacesGroup' % self.switchController, force=True)
                except Exception as e:
                    log.info(e)

                spaces = []
                spacesDict = {}
                for spaceLayout in self.spacesLayouts:
                    driverNode = self.getSpaceDriverNode(spaceLayout)
                    if self.spaceExists(driverNode):
                        log.info('Found the driver')
                        continue
                    displayName = self.getSpaceDisplayName(spaceLayout)
                    if not driverNode:
                        continue
                    else:
                        if not cmds.objExists(driverNode):
                            self.missingValuesWarnings("%s doesn't exist in the scene" % driverNode)
                            return
                        if not displayName:
                            displayName = getShortName(driverNode)
                    spacesDict = {driverNode: displayName}
                    spaces.append(spacesDict)

                if not spaces:
                    return
                self.spaceSwitch(self.drivenNode, self.switchController, self.constraintType, spaces, self.spacesGroup, self.switchAttribute)
            except Exception as e:
                log.exception(e)
                QMessageBox.warning(self, 'Error:', str(e))
                return

            log.info('Space Switch Setup completed successfully!')
        finally:
            cmds.undoInfo(closeChunk=True)

    def getDriversDict(self):
        spacesDict = {}
        for spaceLayout in self.spacesLayouts:
            driverNode = self.getSpaceDriverNode(spaceLayout)
            if not driverNode:
                continue
            displayName = self.getSpaceDisplayName(spaceLayout)
            spacesDict.setdefault(driverNode, displayName)

        return spacesDict

    def missingValuesWarnings(self, msg, lineEdit = ''):
        QMessageBox.warning(self, 'Value Error', msg)
        if lineEdit:
            lineEdit.setFocus()

    def spaceSwitch(self, drivenNode, controller, constraintType, driverSpaces, spacesGrp = None, attrName = 'spaces'):
        constraintsList = {'point': cmds.pointConstraint,
         'orient': cmds.orientConstraint,
         'parent': cmds.parentConstraint}
        constraint = constraintsList[constraintType]
        driverGrp = None
        enumName = []
        for spaceDict in driverSpaces:
            for key, value in spaceDict.iteritems():
                driverGrp = ''
                if attributeExists(key, 'PP_driverGroup'):
                    driverGrp = cmds.listConnections(key + '.PP_driverGroup')
                    if driverGrp:
                        driverGrp = getMDagPath(driverGrp[0])
                if not cmds.objExists(key + '_space'):
                    driverGrp = getMDagPath(cmds.group(empty=True, name=getShortName(key) + '_space'))
                else:
                    driverGrp = getMDagPath(key + '_space')
                addMsgAttr(key, driverGrp.fullPathName(), 'PP_driverGroup')
                try:
                    cmds.pointConstraint(key, driverGrp.fullPathName(), maintainOffset=0)
                    cmds.orientConstraint(key, driverGrp.fullPathName(), maintainOffset=0)
                except RuntimeError as e:
                    log.info(e)

                try:
                    cmds.parent(driverGrp.fullPathName(), spacesGrp)
                except Exception as e:
                    log.info(e)

                if not attributeExists(controller, attrName):
                    cmds.addAttr(controller, longName=attrName, keyable=True, attributeType='enum', enumName=value)
                enumName = cmds.addAttr('%s.%s' % (controller, attrName), query=True, enumName=True)
                enumList = enumName.split(':')
                for enum in enumList:
                    if enum == value:
                        enumList.remove(value)

                newEnum = ':'.join(enumList)
                cmds.addAttr('%s.%s' % (controller, attrName), edit=True, enumName='%s:%s' % (newEnum, value))
                constraintGrp = '%s_%s_space' % (getShortName(drivenNode), getShortName(key))
                constraintGrpOffset = '%s_offset' % constraintGrp
                if not cmds.objExists(constraintGrp):
                    constraintGrp = cmds.group(empty=True, name=constraintGrp)
                    constraintGrpOffset = cmds.group(constraintGrp, name='%s_offset' % constraintGrp)
                addMsgAttr(key, constraintGrp, 'PP_driverNode')
                addMsgAttr(constraintGrp, constraintGrpOffset, 'PP_offsetGrp')
                cmds.delete(cmds.pointConstraint(drivenNode, constraintGrpOffset)[0])
                drivenNodeScale = cmds.xform(drivenNode, query=True, scale=True, worldSpace=True)
                cmds.setAttr(constraintGrpOffset + '.scale', drivenNodeScale[0], drivenNodeScale[1], drivenNodeScale[2], type='double3')
                drivenNodeRot = cmds.xform(drivenNode, query=True, rotation=True, worldSpace=True)
                cmds.xform(constraintGrpOffset, rotation=drivenNodeRot, worldSpace=True)
                cmds.delete(cmds.orientConstraint(drivenNode, constraintGrp)[0])
                try:
                    cmds.parent(constraintGrpOffset, driverGrp.fullPathName())
                except RuntimeError as e:
                    log.info(e)

                constraintNode = constraint(constraintGrp, drivenNode, maintainOffset=True)[0]
                constraintTargetList = constraint(constraintNode, targetList=True, query=True)
                constraintWeightsList = constraint(constraintNode, weightAliasList=True, query=True)
                constraintDriverIdx = constraintTargetList.index(constraintGrp)
                addMsgAttr(drivenNode, constraintNode, 'PP_spaceConstraint')
                cond = '%s_%s_space_COND' % (getShortName(drivenNode), key)
                finalEnumList = cmds.addAttr('%s.%s' % (controller, attrName), query=True, enumName=True)
                finalEnumList = finalEnumList.split(':')
                if not attributeExists(constraintGrp, 'PP_switchNo'):
                    cmds.addAttr(constraintGrp, longName='PP_switchNo', at='long')
                cmds.setAttr('%s.PP_switchNo' % constraintGrp, constraintDriverIdx)
                if not cmds.objExists(cond):
                    cond = cmds.shadingNode('condition', name=cond, asUtility=True)
                try:
                    cmds.connectAttr('%s.PP_switchNo' % constraintGrp, cond + '.secondTerm')
                except RuntimeError as e:
                    log.info(e)

                cmds.setAttr(cond + '.operation', 0)
                cmds.setAttr(cond + '.colorIfTrueR', 1)
                cmds.setAttr(cond + '.colorIfFalseR', 0)
                try:
                    cmds.connectAttr(cond + '.outColorR', '%s.%s' % (constraintNode, constraintWeightsList[constraintDriverIdx]), force=True)
                except Exception as e:
                    log.info(e)

                try:
                    cmds.connectAttr('%s.%s' % (controller, attrName), cond + '.firstTerm', force=True)
                except Exception as e:
                    log.info(e)

                addMsgAttr(controller, drivenNode, 'PP_drivenNode')
                addMsgAttr(constraintGrp, cond, 'PP_condNode')

        stampName = 'PP_spaceDriver'
        if not attributeExists(controller, stampName):
            cmds.addAttr(controller, longName=stampName, at='long', dv=0)
        try:
            cmds.connectAttr('%s.%s' % (controller, attrName), '%s.PP_spaceDriver' % controller, force=True)
        except Exception as e:
            log.info(e)

        return

    def spaceExists(self, driver):
        driversLst = self.getDrivers(self.switchController)
        if not driversLst:
            return
        if driver in driversLst:
            log.info('%s is already an driving space for %s' % (driver, self.drivenNode))
            return driver

    def getDrivers(self, switcherNode):
        constraintTargetsList = self.getConstraintGroups(switcherNode)
        driverNodes = []
        if not constraintTargetsList:
            return
        for target in constraintTargetsList:
            driverN = self.getDriverFromGrp(target)
            if driverN:
                driverNodes.append(driverN)

        return driverNodes

    def getConstraintGroups(self, switcherNode):
        if not attributeExists(switcherNode, 'PP_drivenNode'):
            return
        drivenNode = cmds.listConnections(switcherNode + '.PP_drivenNode')
        if not drivenNode:
            return
        spaceConstraint = cmds.listConnections(drivenNode[0] + '.PP_spaceConstraint')
        if not spaceConstraint:
            return
        spaceConstraint = spaceConstraint[0]
        constraintsList = {'pointConstraint': cmds.pointConstraint,
         'orientConstraint': cmds.orientConstraint,
         'parentConstraint': cmds.parentConstraint}
        constraint = constraintsList[cmds.nodeType(spaceConstraint)]
        constraintTargetsList = constraint(spaceConstraint, targetList=True, query=True)
        return constraintTargetsList

    def getDriverFromGrp(self, grp):
        if not attributeExists(grp, 'PP_driverNode'):
            return
        driverN = cmds.listConnections(grp + '.PP_driverNode')
        if driverN:
            return driverN[0]

    def getDisplayName(self, grp, switcher):
        if not attributeExists(grp, 'PP_switchNo'):
            log.info('No switch No found, skipping the %s driver' % grp)
            return
        switchNo = cmds.getAttr('%s.PP_switchNo' % grp)
        spaceCon = cmds.listConnections(switcher + '.PP_spaceDriver', p=True)
        enumNames = cmds.addAttr(spaceCon[0], query=True, enumName=True)
        enumList = enumNames.split(':')
        return enumList[switchNo]

    def selectFieldNodes(self, editField):
        node = str(editField.text())
        if node:
            cmds.select(node)
        return node

    def loadSpaceDriverNode(self, editField):
        node = getSelected(editField)
        if not node:
            return
        if not attributeExists(node, 'PP_spaceDriver'):
            log.info('No space switch found on this Node %s' % node)
            QMessageBox.warning(self, 'Value Error', 'No space switch found on this Node %s' % node)
            editField.clear()
            return
        return node

    def fillInfoFromField(self, lineEdit):
        try:
            if not cmds.objExists(lineEdit):
                return
            if not attributeExists(lineEdit, 'PP_drivenNode'):
                return
            switcherNode = self.switchController
            drivenNode = cmds.listConnections(switcherNode + '.PP_drivenNode')
            if not drivenNode:
                return
            drivenNode = drivenNode[0]
            self.drivenNodeEdit.setText(drivenNode)
            spaceAttr = ''
            spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)
            if spaceCon:
                spaceAttr = spaceCon[0].split('.')[-1]
            self.attributeNameEdit.setText(spaceAttr)
            spaceConstraint = cmds.listConnections(drivenNode + '.PP_spaceConstraint')
            if spaceConstraint:
                spaceConstraint = spaceConstraint[0]
            constraintType = cmds.nodeType(spaceConstraint)
            if constraintType == 'parentConstraint':
                self.constraintComboBox.setCurrentIndex(0)
            if constraintType == 'pointConstraint':
                self.constraintComboBox.setCurrentIndex(1)
            if constraintType == 'orientConstraint':
                self.constraintComboBox.setCurrentIndex(2)
            spacesGrp = cmds.listConnections(switcherNode + '.PP_spacesGroup')
            if spacesGrp:
                cmds.select(spacesGrp[0])
                getSelected(self.spacesEdit)
            else:
                self.spacesEdit.setText('')
        except Exception as e:
            log.info("Couldn't get previous space setup info!\n%s" % e)

    def clearCreateSpacesInfo(self):
        for lineEdit in [self.switchControllerEdit,
         self.attributeNameEdit,
         self.spacesEdit,
         self.drivenNodeEdit]:
            lineEdit.clear()

    def clearSpacesFrame(self):
        spaces = self.spacesLayouts
        if not spaces:
            return
        else:
            for frame in spaces:
                frame.setParent(None)
                frame.deleteLater()

            return

    def loadAvailableSpaces(self):
        switcherNode = self.loadSpaceDriverNode(self.bakeSwitcherNodeEdit)
        if not switcherNode:
            self.resetBakeTab()
            return
        spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)
        enumNames = cmds.addAttr(spaceCon[0], query=True, enumName=True)
        enumList = enumNames.split(':')
        self.availableSpacesComboBox.clear()
        self.availableSpacesComboBox.addItems(enumList)

    def useTimeline(self, checked = False):
        if not checked:
            return
        frames = switchUtils.getTimelineRange()
        startFrame = frames[0]
        endFrame = frames[-1]
        self.bakeStartFrameSpin.setValue(startFrame)
        self.bakeEndFrameSpin.setValue(endFrame)
        self.setBakeWidgetsState(True)

    def usePlayback(self, checked = False):
        if not checked:
            return
        frames = switchUtils.getPlaybackRange()
        startFrame = frames[0]
        endFrame = frames[-1]
        self.bakeStartFrameSpin.setValue(startFrame)
        self.bakeEndFrameSpin.setValue(endFrame)
        self.setBakeWidgetsState(True)

    def useCustomFrameRange(self, checked = False):
        if not checked:
            self.setBakeWidgetsState(False)
            return
        self.setBakeWidgetsState(True)

    def setBakeWidgetsState(self, state):
        for bakeWid in [self.bakeEndFrameLabel,
         self.bakeEndFrameSpin,
         self.bakeStartFrameLabel,
         self.bakeStartFrameSpin]:
            bakeWid.setEnabled(state)

    def bakeSpace(self):
        switcherNode = str(self.bakeSwitcherNodeEdit.text())
        if not switcherNode:
            return
        cmds.undoInfo(openChunk=True)
        try:
            try:
                if self.bakeStartFrameSpin.value() == self.bakeEndFrameSpin.value():
                    log.warning('The start or the end frame is missing! Please add a start and end frames.')
                    return
                if not self.bakeStartFrameSpin.isEnabled():
                    return
                frameRange = [self.bakeStartFrameSpin.value(), self.bakeEndFrameSpin.value()]
                targetSpace = self.availableSpacesComboBox.currentIndex()
                spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)[0]
                spaceCon = spaceCon.split('.')[-1]
                switchUtils.spaceSwitchBake(switcherNode, spaceCon, targetSpace, frameRange)
            except Exception as e:
                log.exception(e)
                return

            log.info('Space successfully baked!')
        finally:
            cmds.undoInfo(closeChunk=True)

    def resetBakeTab(self):
        for wid in [self.bakeSwitcherNodeEdit, self.availableSpacesComboBox]:
            wid.clear()

    def loadEditUI(self):
        try:
            self.resetEditTab()
            switcherNode = self.loadSpaceDriverNode(self.editSpaceNodeEdit)
            if not switcherNode:
                return
            self._editSwitcherNode = getMDagPath(switcherNode)
            drivenNode = cmds.listConnections(switcherNode + '.PP_drivenNode')
            if drivenNode:
                self.editDrivenNodeEdit.setText(drivenNode[0])
                self._editDrivenNode = getMDagPath(drivenNode[0])
            constraint = cmds.listConnections(drivenNode[0] + '.PP_spaceConstraint')
            if constraint:
                self.editDrivingConstEdit.setText(constraint[0])
                self._editConstraintNode = getMDagPath(constraint[0])
            else:
                return
            grpLst = self.getConstraintGroups(switcherNode)
            for i, grp in enumerate(grpLst):
                driver = self.getDriverFromGrp(grp)
                if not driver:
                    continue
                driverFrame = DriversFrame(self)
                self.driversContent.layout().addWidget(driverFrame)
                driverFrame.constraintGrp = grp
                driverFrame.driverNodeEdit.setText(driver)
                driverFrame.driverDisplayEdit.setText(self.getDisplayName(grp, switcherNode))
                driverFrame.orderNoLabel.setText('%s-' % i)
                driverFrame.removeButton.clicked.connect(partial(self.deleteSpace, driverFrame))
                driverFrame.driverDisplayEdit.editingFinished.connect(partial(self.editDisplayName, driverFrame))
                self._editFrames.append(driverFrame)

        except Exception as e:
            log.exception(e)

    def createParentGroup(self):
        if not self.switchController:
            return
        else:
            drivenNode = None
            if attributeExists(self.switchController, 'PP_drivenNode'):
                drivenNode = cmds.listConnections(self.switchController + '.PP_drivenNode')
            constraint = None
            if drivenNode:
                constraint = cmds.listConnections(drivenNode[0] + '.PP_spaceConstraint')
            if constraint:
                QMessageBox.warning(self, 'Warnning', '%s already has a space group %s' % (getShortName(self.switchController), drivenNode[0]))
                return
            offsetGroup = cmds.group(n='%s_space_GRP' % getShortName(self.switchController), world=True, empty=True)
            cmds.delete(cmds.pointConstraint(self.switchController, offsetGroup)[0])
            drivenNodeRot = cmds.xform(self.switchController, query=True, rotation=True, worldSpace=True)
            cmds.xform(offsetGroup, rotation=drivenNodeRot, worldSpace=True)
            drivenNodeScale = cmds.xform(self.switchController, query=True, scale=True, worldSpace=True)
            cmds.xform(offsetGroup, scale=drivenNodeScale, worldSpace=True)
            switchController = getMDagPath(self.switchController)
            controllerParent = cmds.listRelatives(self.switchController, parent=True)
            if controllerParent:
                cmds.parent(offsetGroup, controllerParent[0])
            cmds.parent(self.switchController, offsetGroup)
            cmds.select(switchController.fullPathName())
            getSelected(self.switchControllerEdit)
            self.drivenNodeEdit.setText(offsetGroup)
            return offsetGroup

    def editSpacesSetup(self):
        try:
            if not self._editFrames:
                return
            constraintsList = {'pointConstraint': cmds.pointConstraint,
             'orientConstraint': cmds.orientConstraint,
             'parentConstraint': cmds.parentConstraint}
            orderedDrivers = range(len(self._editFrames))
            orderedDisplayNames = range(len(self._editFrames))
            for frame in self._editFrames:
                frame.orderNoLabel.setText('%s-' % self.driversContent.layout().indexOf(frame))
                idx = self.driversContent.layout().indexOf(frame)
                orderedDrivers[idx] = frame.constraintGrp
                orderedDisplayNames[idx] = frame.displayName
                cmds.setAttr(frame.constraintGrp + '.PP_switchNo', idx)

            drivenNode = self._editDrivenNode.fullPathName()
            constraint = self._editConstraintNode.fullPathName()
            constraintType = constraintsList[cmds.nodeType(constraint)]
            switcherNode = self._editSwitcherNode.fullPathName()
            spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)[0]
            constraintType(orderedDrivers[1:], drivenNode, remove=True, e=True)
            constraintType(orderedDrivers[1:], drivenNode)
            cmds.addAttr(spaceCon, edit=True, enumName=':'.join(orderedDisplayNames))
            constraintWeightsList = constraintType(constraint, weightAliasList=True, query=True)
            for i, driver in enumerate(orderedDrivers):
                cond = cmds.listConnections(driver + '.PP_condNode')[0]
                try:
                    cmds.connectAttr(cond + '.outColorR', '%s.%s' % (constraint, constraintWeightsList[i]), force=True)
                except RuntimeError as e:
                    log.info(e)

        except Exception as e:
            log.exception(e)

    def editDisplayName(self, widget):
        try:
            switcherNode = self._editSwitcherNode.fullPathName()
            spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)[0]
            idx = self.driversContent.layout().indexOf(widget)
            enumName = cmds.addAttr(spaceCon, query=True, enumName=True)
            enumList = enumName.split(':')
            enumList[idx] = widget.displayName
            newEnum = ':'.join(enumList)
            spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)[0]
            cmds.addAttr(spaceCon, edit=True, enumName=newEnum)
        except Exception as e:
            log.exception(e)

    def deleteSpace(self, widget):
        driver = widget.driverNode
        msgBox = QMessageBox(self)
        msgBox.setText('Delete Selected Space!')
        msgBox.setInformativeText('Are you sure you want to delete the %s space?' % driver)
        msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        msgBox.setDefaultButton(QMessageBox.Save)
        ret = msgBox.exec_()
        if ret == QMessageBox.Yes:
            self._deleteSpace(widget)
        else:
            return False

    def _deleteSpace(self, widget):
        cmds.undoInfo(openChunk=True)
        try:
            try:
                driverGrp = widget.constraintGrp
                driver = widget.driverNode
                widget.setParent(None)
                widget.deleteLater()
                self._editFrames.remove(widget)
                driverCond = cmds.listConnections(driverGrp + '.PP_condNode')
                driverOffsetGrp = cmds.listConnections(driverGrp + '.PP_offsetGrp')
                cmds.delete(driverCond, driverOffsetGrp)
                self.editSpacesSetup()
            except Exception as e:
                log.exception(e)
                return

            log.info('%s Space removed successfully!' % driver)
        finally:
            cmds.undoInfo(closeChunk=True)

        return

    def selectAllDrivers(self):
        if not self._editFrames:
            return
        drivers = []
        for frame in self._editFrames:
            drivers.append(frame.driverNode)

        cmds.select(drivers)
        return drivers

    def deleteSetup(self):
        if not self._editFrames:
            return
        msgBox = QMessageBox(self)
        msgBox.setText('This action may be undoable!')
        msgBox.setInformativeText('Are you sure you want to delete this setup?')
        msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        msgBox.setDefaultButton(QMessageBox.Save)
        ret = msgBox.exec_()
        if ret == QMessageBox.Yes:
            self._deleteSetup()
        elif ret == QMessageBox.No:
            return False

    def _deleteSetup(self):
        cmds.undoInfo(openChunk=True)
        try:
            try:
                for frame in self._editFrames:
                    frame.setParent(None)
                    frame.deleteLater()

                switcherNode = self._editSwitcherNode.fullPathName()
                for grp in self.getConstraintGroups(switcherNode):
                    driverCond = cmds.listConnections(grp + '.PP_condNode')
                    driverOffsetGrp = cmds.listConnections(grp + '.PP_offsetGrp')
                    cmds.delete(driverCond, driverOffsetGrp)

                spaceCon = cmds.listConnections(switcherNode + '.PP_spaceDriver', p=True)[0]
                cmds.deleteAttr(spaceCon)
                self._editFrames = []
                self._editSwitcherNode = None
                self._editConstraintNode = None
                self._editDrivenNode = None
            except Exception as e:
                log.exception(e)
                return

            log.info('Space Switch Setup deleted successfully!')
        finally:
            cmds.undoInfo(closeChunk=True)

        return

    def resetEditTab(self):
        for lineEdit in [self.editDrivenNodeEdit, self.editDrivingConstEdit, self.editSpaceNodeEdit]:
            lineEdit.clear()

        driversLayout = self.driversContent.layout()
        self._editFrames = []
        self._editSwitcherNode = None
        self._editConstraintNode = None
        self._editDrivenNode = None
        try:
            for i in reversed(range(driversLayout.count())):
                widget = driversLayout.itemAt(i).widget()
                widget.setParent(None)
                widget.deleteLater()

        except IndexError:
            log.info('No drivers found to remove')
        except Exception as e:
            log.exception(e)

        return

    def closeEvent(self, event):
        self.writeSettings()

    def readSettings(self):
        settings = QSettings(QSettings.IniFormat, QSettings.UserScope, 'PurplePuppet_INC', 'PP_SpaceSwitchTool')
        self.settingsFile = settings.fileName()
        log.info('PP_spaceSwitchTool settings file read from %s' % settings.fileName())
        self.drivenNode_ini = settings.value('drivenNode', '')
        self.switchNode_ini = settings.value('switchNode', '')
        self.switchAttr_ini = settings.value('switchAttr', '')
        self.spacesGrp_ini = settings.value('spacesGrp', '')
        self.constraintType_ini = int(settings.value('constraintType', 0))
        self.spacesDict_ini = settings.value('drivers', {})

    def writeSettings(self):
        settings = QSettings(self.settingsFile, QSettings.IniFormat)
        self.drivenNode_ini = self.drivenNode
        self.switchNode_ini = self.switchController
        self.switchAttr_ini = self.switchAttribute
        self.spacesGrp_ini = self.spacesGroup
        self.constraintType_ini = int(self.constraintComboBox.currentIndex())
        self.spacesDict_ini = self.getDriversDict()
        settings.setValue('drivenNode', self.drivenNode_ini)
        settings.setValue('switchNode', self.switchNode_ini)
        settings.setValue('switchAttr', self.switchAttribute)
        settings.setValue('spacesGrp', self.spacesGrp_ini)
        settings.setValue('constraintType', self.constraintType_ini)
        settings.setValue('drivers', self.spacesDict_ini)
        log.info('PP_spaceSwitchTool settings file saved at %s' % settings.fileName())

    def loadRecentSettings(self):
        self.drivenNodeEdit.setText(self.drivenNode_ini)
        self.switchControllerEdit.setText(self.switchNode_ini)
        self.attributeNameEdit.setText(self.switchAttr_ini)
        self.spacesEdit.setText(self.spacesGrp_ini)
        self.constraintComboBox.setCurrentIndex(self.constraintType_ini)
        if self.spacesDict_ini:
            for driver, displayName in self.spacesDict_ini.items():
                spaceFrame = self.addSpaceLayout(False)
                spaceFrame.driverNodeEdit.setText(driver)
                spaceFrame.displayNameEdit.setText(displayName)

    def resetSettings(self, *args):
        settings = QSettings(self.settingsFile, QSettings.IniFormat)
        settings.clear()

    def addCallBacks(self):
        self.callback_list.append(OpenMaya.MSceneMessage.addCallback(OpenMaya.MSceneMessage.kMayaExiting, self.resetSettings))

    def removeCallbacks(self):
        for callback in self.callback_list:
            OpenMaya.MMessage.removeCallback(callback)


class DropFrame(QWidget):
    dropped = Signal(QWidget, int)

    def __init__(self, parent = None):
        super(DropFrame, self).__init__(parent)
        self.setAcceptDrops(True)
        self.contentsVLO = QVBoxLayout()
        self.contentsVLO.setAlignment(Qt.AlignTop)
        self.setLayout(self.contentsVLO)
        self.setObjectName('spaces_Widget')

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-dropFrame'):
            event.setDropAction(Qt.MoveAction)
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        if event.mimeData().hasFormat('application/x-dropFrame'):
            event.setDropAction(Qt.MoveAction)
            event.accept()
            cardSource = event.source()
            cardSource.moveSpaceFrame()
            for i in range(cardSource.moveOffset()):
                self.moveInLayout(cardSource, cardSource.moveDirection())

            cardSource.moveUpdate()
            self.update()
            cardSource.saveGeometry()
        else:
            event.ignore()

    def moveInLayout(self, widget, direction):
        index = self.contentsVLO.indexOf(widget)
        if direction == 'UP' and index == 0:
            return 0
        if direction == 'DOWN' and index == self.contentsVLO.count() - 1:
            return 0
        newIndex = index
        if direction == 'UP':
            newIndex = index - 1
        else:
            newIndex = index + 1
        self.contentsVLO.removeWidget(widget)
        self.contentsVLO.insertWidget(newIndex, widget)
        self.dropped.emit(widget, newIndex)
        return True


class DragFrame(QFrame):

    def __init__(self, parent = None):
        super(DragFrame, self).__init__(parent)
        self.setAutoFillBackground(True)
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMinimumSize(QSize(350, 50))
        self.contentsHLO = QHBoxLayout()
        self.setLayout(self.contentsHLO)

    def mousePressEvent(self, event):
        self.dragStartPosition = event.pos()
        self.oldX = self.geometry().x()
        self.oldY = self.geometry().y()
        mousePosition = self.mapFromGlobal(QtGui.QCursor.pos())
        self.mouseClickX = mousePosition.x()
        self.mouseClickY = mousePosition.y()
        self.setStyleSheet(self.styleSheetPressed)

    def isMinimumDistanceRiched(self, event):
        return (event.pos() - self.dragStartPosition).manhattanLength() >= QApplication.startDragDistance()

    def mouseMoveEvent(self, event):
        mimeData = QMimeData()
        itemData = QByteArray()
        mimeData.setData('application/x-dropFrame', itemData)
        pixmap = QtGui.QPixmap.grabWidget(self)
        painter = QtGui.QPainter(pixmap)
        painter.setCompositionMode(painter.CompositionMode_DestinationIn)
        painter.fillRect(pixmap.rect(), QtGui.QColor(0, 0, 0, 127))
        painter.end()
        drag = QtGui.QDrag(self)
        drag.setMimeData(mimeData)
        drag.setPixmap(pixmap)
        drag.setHotSpot(self.dragStartPosition)
        drag.exec_(Qt.MoveAction)

    def mouseReleaseEvent(self, event):
        self.moveUpdate()

    def moveUpdate(self):
        self.update()
        parentWid = self.parentWidget().layout()
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setStyleSheet(self.styleSheetDefault)
        parentWid.update()
        self.saveGeometry()

    def moveSpaceFrame(self):
        mousePosition = self.mapFromGlobal(QtGui.QCursor.pos())
        mousePosY = mousePosition.y()
        yValue = mousePosY - self.mouseClickY + self.oldY
        bottomBorder = self.parentWidget().geometry().height() - self.geometry().height()
        if yValue < 0:
            yValue = 0
        elif yValue > bottomBorder:
            yValue = bottomBorder
        self.move(self.oldX, yValue)

    def moveDirection(self):
        yPos = self.geometry().y()
        self.direction = ''
        if self.oldY > yPos:
            self.direction = 'UP'
        elif self.oldY < yPos:
            self.direction = 'DOWN'
        return self.direction

    def moveOffset(self):
        yPos = self.geometry().y()
        offset = 0
        if self.oldY > yPos:
            offset = self.oldY - yPos
        elif self.oldY < yPos:
            offset = yPos - self.oldY
        count = offset / self.height()
        return count


class SpacesFrame(DragFrame):

    def __init__(self, parent = None):
        super(SpacesFrame, self).__init__(parent)
        self.setObjectName('spaces_Frame')
        self.driverNodeLabel = QLabel('&Driver Node:')
        self.driverNodeEdit = QLineEdit()
        self.driverNodeEdit.setObjectName('driverNode_LineEdit')
        self.driverNodeLabel.setBuddy(self.driverNodeEdit)
        self.driverNodeButton = QPushButton('<<')
        self.driverNodeEdit.setToolTip('Add a driver node.')
        self.driverNodeLabel.setToolTip('The driver node space.')
        self.driverNodeButton.setToolTip('Select the driver node in the scene and press to add to the name field.')
        self.displayNameLabel = QLabel('&Display Name:')
        self.displayNameEdit = QLineEdit()
        self.displayNameEdit.setObjectName('displayName_LineEdit')
        self.displayNameLabel.setBuddy(self.displayNameEdit)
        self.displayNameEdit.setToolTip('Add the driver node display name in the switch attribute.')
        self.displayNameLabel.setToolTip('The driver node display name in the switch attribute..')
        self.removeButton = QPushButton('X')
        self.removeButton.setToolTip('Delete the current space.')
        self.removeButton.setStyleSheet('color: #ac3737; font-weight:bold;font-size: 10pt')
        self.removeButton.setMinimumWidth(30)
        for wid in [self.driverNodeLabel,
         self.driverNodeEdit,
         self.driverNodeButton,
         self.displayNameLabel,
         self.displayNameEdit,
         self.removeButton]:
            self.contentsHLO.addWidget(wid)

        self.contentsHLO.setStretch(1, 0)
        self.driverNodeButton.clicked.connect(partial(getSelected, self.driverNodeEdit))
        self.removeButton.clicked.connect(self.deleteSpace)
        self.styleSheetDefault = '\n            #spaces_Frame{background-color:#404040;}\n            #spaces_Frame:hover {background-color:#474747;}\n            QLineEdit{background-color: #5e718a;}\n            QLabel {font-weight:bold; color:#19dfbc;}\n            '
        self.styleSheetPressed = '\n            #spaces_Frame{background-color:#373737; border-style:inset; border-width:2px; border-color:#2b2e31}\n            QLineEdit{background-color: #5e718a;}\n            QLabel {font-weight:bold; color:#19dfbc;}\n            '
        self.setStyleSheet(self.styleSheetDefault)

    @property
    def driverNode(self):
        return str(self.driverNodeEdit.text())

    @property
    def displayName(self):
        return str(self.displayNameEdit.text())

    def deleteSpace(self):
        self.setParent(None)
        self.deleteLater()
        return


class DriversFrame(DragFrame):

    def __init__(self, parent = None):
        super(DriversFrame, self).__init__(parent)
        self.setObjectName('drivers_Frame')
        self.driverNodeLabel = QLabel('&Driver Node:')
        self.driverNodeEdit = QLineEdit()
        self.driverNodeEdit.setObjectName('driverNode_LineEdit')
        self.driverNodeLabel.setBuddy(self.driverNodeEdit)
        self.driverNodeEdit.setReadOnly(True)
        self.constraintGrp = None
        self.orderNoLabel = QLabel()
        self.orderNoLabel.setMaximumWidth(50)
        self.orderNoLabel.setEnabled(False)
        self.driverDispalyLabel = QLabel('&Display Name:')
        self.driverDisplayEdit = QLineEdit()
        self.driverDisplayEdit.setObjectName('driverDispaly_LineEdit')
        self.driverDispalyLabel.setBuddy(self.driverDisplayEdit)
        self.removeButton = QPushButton('Delete')
        self.removeButton.setToolTip('Delete the current space.')
        self.removeButton.setStyleSheet('color: #ac3737; font-weight:bold;font-size: 10pt')
        self.removeButton.setMinimumWidth(30)
        for wid in [self.orderNoLabel,
         self.driverNodeLabel,
         self.driverNodeEdit,
         self.driverDispalyLabel,
         self.driverDisplayEdit,
         self.removeButton]:
            self.contentsHLO.addWidget(wid)

        self.contentsHLO.setStretch(1, 0)
        self.styleSheetDefault = '\n            #drivers_Frame{background-color:#404040;}\n            #drivers_Frame:hover {background-color:#474747;}\n            QLineEdit{background-color: #5e718a;}\n            QLabel {font-weight:bold; color:#19dfbc;}\n            '
        self.styleSheetPressed = '\n            #drivers_Frame{background-color:#373737; border-style:inset; border-width:2px; border-color:#2b2e31}\n            QLineEdit{background-color: #5e718a;}\n            QLabel {font-weight:bold; color:#19dfbc;}\n            '
        self.setStyleSheet(self.styleSheetDefault)
        return

    @property
    def driverNode(self):
        return str(self.driverNodeEdit.text())

    @property
    def displayName(self):
        return str(self.driverDisplayEdit.text())


def getMDagPath(node):
    newNode = node
    if node.startswith('|'):
        newNode = node[1:]
    sel = cmds.ls(newNode)[0]
    dagpath = OpenMaya.MDagPath()
    selList = OpenMaya.MSelectionList()
    selList.add(sel)
    selList.getDagPath(0, dagpath)
    return dagpath


def getSelected(field):
    sel = cmds.ls(sl=True, l=True)
    if not sel:
        cmds.warning('No selection found!')
        return
    if len(sel) > 1:
        cmds.warning('More than one selection found. taking first selection!')
    newNode = sel[0]
    if newNode.startswith('|'):
        newNode = newNode[1:]
    field.clear()
    field.setText(newNode)
    return newNode


def attributeExists(node, attr):
    return cmds.attributeQuery(attr, node=node, exists=True)


def getShortName(node):
    return node.split('|')[-1].split(':')[-1]


def addMsgAttr(node01, node02, attrName, force = True):
    if not attributeExists(node01, attrName):
        cmds.addAttr(node01, longName=attrName, attributeType='message')
    if not attributeExists(node02, attrName):
        cmds.addAttr(node02, longName=attrName, attributeType='message')
    try:
        cmds.connectAttr('%s.%s' % (node01, attrName), '%s.%s' % (node02, attrName), f=force)
    except StandardError as error:
        log.info(error)