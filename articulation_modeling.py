# -- coding: UTF-8 --
import sys
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import os
import numpy as np
import json
import math
import copy
from PIL import Image
from annotator_window import Annotator

import ctypes

ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID()

__appname__ = 'ArticulationModeling'


class AnnotationLabel(QLabel):
    def __init__(self, layout, annotator_window, mainwindow):
        super(AnnotationLabel, self).__init__(layout)
        self.annotator = annotator_window
        self.mainwindow = mainwindow
        self.scale = None

    # def keyPressEvent(self, QKeyEvent):
    #     self.annotator.keyboard_event(QKeyEvent.key())
    #
    #     self.mainwindow.show_img()

    def update_scale(self, scale):
        self.scale = scale


class ComBox(QComboBox):
    def __init__(self):
        super(ComBox, self).__init__()


class ClickableImage(QLabel):
    def __init__(self, width=0, height=0, pixmap=None, image_id='', mainwindow=None):
        super(QLabel, self).__init__()

        self.width = width
        self.height = height
        self.pixmap = pixmap
        self.image_id = os.path.basename(image_id)
        self.mainwindow = mainwindow
        self.border = False

        if self.width and self.height:
            self.resize(self.width, self.height)
        if self.pixmap and image_id:
            pixmap = self.pixmap.scaled(QSize(self.width, self.height), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.setPixmap(pixmap)
            self.setAlignment(Qt.AlignCenter)

    clicked = pyqtSignal(object)
    rightClicked = pyqtSignal(object)

    def update_annotator_model(self):
        if not self.mainwindow.annotator_window.model_to_be_annotated:
            QMessageBox.warning(self.mainwindow, 'warning', 'load RGBD images first!')
            return

        self.mainwindow.annotator_window.update_model(self.image_id.split('.')[0])

        # self.mainwindow.show_img()
        self.draw_border()

    def draw_border(self):
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(5)
        self.setStyleSheet('border-width: 5px;border-style: solid;border-color: rgb(255, 0, 0);')
        # self.setStyleSheet('border-color: rgb(255, 0, 0)')
        self.border = True

    def clear_border(self):
        if self.border:
            self.setLineWidth(0)
            self.setStyleSheet('border-width: 0')
            self.border = False


class ImageBrower(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

    def mousePressEvent(self, QMouseEvent):
        if len(self.img_x_range) != self.layout().count():
            self.find_widget_pos(self.layout())
        index = None
        x = QMouseEvent.x()
        temp_img_range = copy.copy(self.img_x_range)
        temp_img_range.append(1e6)
        for i in range(self.img_num):
            if i + 1 <= self.img_num and temp_img_range[i] < x and temp_img_range[i + 1] > x:
                index = i
                break
        if index is not None:
            for i in range(self.img_num):
                self.layout().itemAt(i).widget().clear_border()
            self.layout().itemAt(index).widget().update_annotator_model()
            # self.layout().itemAt(index).widget().draw_border()
        # print('click')

    def find_widget_pos(self, layout):
        self.img_num = layout.count()
        self.img_x_range = [None] * self.img_num
        for i in range(self.img_num):
            widget = layout.itemAt(i).widget()
            left_x = widget.x()
            # width = widget.width
            range_i_x = left_x
            self.img_x_range[i] = range_i_x


class RadioBox(QWidget):
    def __init__(self, botton_names=(), size=(50, 100), mainwindow=None):
        super(RadioBox, self).__init__()
        layout = QHBoxLayout()  # 实例化一个布局

        self.btn_list = []
        for i, botton_name in enumerate(botton_names):
            self.btn_list.append(QRadioButton(botton_name)) # 实例化一个选择的按钮
            if i == 0:
                self.btn_list[i].setChecked(True) # 设置按钮点点击状态
            self.btn_list[i].toggled.connect(self.btnstate)  # 绑定点击事件
            self.btn_list[i].setStyleSheet(
                "background-color: rgba(255, 255, 255, 0);font-size:25px;font-weight:bold;font-family:Times New Roman;")
            layout.addWidget(self.btn_list[i])  # 布局添加组件

        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)
        self.setFixedSize(size[0], size[1])

        self.mainwindow = mainwindow
        self.current_joint_type = 'prismatic'

    def btnstate(self):  # 自定义点击事件函数
        sender = self.sender()
        if sender.text() == "prismatic" and sender.isChecked() == True:
            self.current_joint_type = 'prismatic'
        if sender.text() == "revolute" and sender.isChecked() == True:
            self.current_joint_type = 'revolute'


class ProcedureRadioBox(QWidget):
    def __init__(self, botton_names=(), size=(50, 100), mainwindow=None):
        super(ProcedureRadioBox, self).__init__()
        layout = QHBoxLayout()  # 实例化一个布局

        self.mainwindow = mainwindow
        func_list = [self.mainwindow.begin_axis_align,
                     self.mainwindow.begin_part_segmentation,
                     self.mainwindow.begin_joint_annotation]

        self.btn_list = []
        self.btn_ann_list = []
        for i, botton_name in enumerate(botton_names):
            self.btn_list.append(QRadioButton(botton_name)) # 实例化一个选择的按钮
            if i == 0:
                self.btn_list[i].setChecked(True) # 设置按钮点点击状态
            self.btn_list[i].toggled.connect(self.btnstate)  # 绑定点击事件
            self.btn_list[i].setStyleSheet(
                "background-color: rgba(255, 255, 255, 0);font-size:30px;font-weight:bold;font-family:Times New Roman;")
            layout.addWidget(self.btn_list[i])  # 布局添加组件

            self.btn_ann_list.append(QToolButton(self))
            self.btn_ann_list[i].setIcon(QIcon('icon/label.png'))
            self.btn_ann_list[i].setIconSize(QSize(32, 32))
            self.btn_ann_list[i].setCursor(QtCore.Qt.PointingHandCursor)
            self.btn_ann_list[i].clicked.connect(func_list[i])
            layout.addWidget(self.btn_ann_list[i])

        layout.setSpacing(50)
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)
        self.setFixedSize(size[0], size[1])

    def btnstate(self):
        sender = self.sender()
        if sender.text() == "Axis Align" and sender.isChecked() == True:
            self.mainwindow.annotator_window.current_ann_stage = 'Axis Align'
            if self.mainwindow.annotator_window.demo_img_axis_align is not None:
                self.mainwindow.show_img(self.mainwindow.annotator_window.demo_img_axis_align)
        if sender.text() == "Part Segmentation" and sender.isChecked() == True:
            self.mainwindow.annotator_window.current_ann_stage = 'Part Segmentation'
            if self.mainwindow.annotator_window.demo_img_part_segmentation is not None:
                self.mainwindow.show_img(self.mainwindow.annotator_window.demo_img_part_segmentation)
        if sender.text() == "Joint Annotation" and sender.isChecked() == True:
            self.mainwindow.annotator_window.current_ann_stage = 'Joint Annotation'
            if self.mainwindow.annotator_window.demo_img_joint_annotation is not None:
                self.mainwindow.show_img(self.mainwindow.annotator_window.demo_img_joint_annotation)


class MainWindow(QMainWindow, QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 窗口大小
        self.resize(1800, 1200)
        # 背景和图标
        palette1 = QPalette()
        # palette1.setColor(self.backgroundRole(), QColor(192,253,123))
        palette1.setBrush(self.backgroundRole(), QBrush(QPixmap('./icon/background.png')))
        self.setPalette(palette1)
        self.setWindowIcon(QIcon('./icon/tool_icon.png'))

        self.layout_widget = QWidget(self)


        # textEdit = QTextEdit()
        self.setCentralWidget(self.layout_widget)

        # 菜单栏
        self.menubar = self.menuBar()
        self.file_menu = self.menubar.addMenu('File')
        self.import_action = QAction('Import...')
        self.import_action.triggered.connect(self.import_model)
        self.file_menu.addAction(self.import_action)

        self.file_menu.addSeparator()

        self.export_action = QAction('Export URDF...')
        self.export_action.triggered.connect(self.export_urdf)
        self.file_menu.addAction(self.export_action)

        self.file_menu.addSeparator()

        self.help_action = QAction('Help')
        self.help_action.triggered.connect(self.show_instruction)
        self.file_menu.addAction(self.help_action)

        self.file_menu.addSeparator()

        self.exit_action = QAction('Quit')
        self.exit_action.setShortcut("Ctrl+Q")
        self.exit_action.triggered.connect(qApp.quit)
        self.file_menu.addAction(self.exit_action)

        #combo.activated[str].connect(self.onActivated)
        selections = ['category1', 'category2']
        self.combo = QComboBox(self)
        self.combo.addItems(selections)
        self.current_selection = selections[0]
        self.combo.setStyleSheet("font-size:25px;font-family:Times New Roman;")
        self.combo.setGeometry(0, 30, int(150), int(30))
        self.combo.currentIndexChanged.connect(self.selectionchange)

        self.scroll_area_images = QScrollArea(self)
        self.scroll_area_images.setWidgetResizable(True)
        self.scrollAreaWidgetContents = ImageBrower()
        self.scrollAreaWidgetContents.setObjectName('scrollAreaWidgetContends')
        self.gridLayout = QGridLayout(self.scrollAreaWidgetContents)
        self.scroll_area_images.setWidget(self.scrollAreaWidgetContents)
        self.scroll_area_images.setGeometry(0, 60, int(1800), int(200))
        self.col = 0
        self.display_image_size = 160
        self.start_img_viewer()

        self.tools_widget = QWidget(self)
        self.tools_widget.setObjectName('ToolsContent')

        self.tools_layout = QVBoxLayout(self.tools_widget)
        self.tools_widget.setGeometry(QtCore.QRect(50, 250, 400, 800))

        # articulation tree
        self.articulation_tree = QTreeWidget(self.tools_widget)
        # self.articulation_tree.setGeometry(20, 780, int(450), int(400))
        self.articulation_tree.setStyleSheet(
            "font-size:22px;font-weight:bold;font-family:Times New Roman;")
        self.articulation_tree.setHeaderLabel('')
        self.articulation_tree.setContextMenuPolicy(Qt.CustomContextMenu)  # 打开右键菜单的策略
        self.articulation_tree.customContextMenuRequested.connect(self.right_click_tree)  # 绑定事件
        self.articulation_item_list = []

        # test animation button
        self.test_save_widget = QWidget(self.tools_widget)
        self.test_save_widget.setFixedSize(380, 100)
        self.horizontalLayout_test_save = QHBoxLayout(self.test_save_widget)

        self.btn_animation = QLabel(self.test_save_widget)
        self.btn_animation.setStyleSheet(
            "background-color: rgba(255, 255, 255, 0);font-size:30px;font-weight:bold;font-family:Times New Roman;")
        self.btn_animation.setText("play")
        self.btn_animation.setAlignment(Qt.AlignCenter)
        self.btn_animation_icon = QPushButton(self.test_save_widget)
        self.btn_animation_icon.setIcon(QIcon('icon/play.png'))
        self.btn_animation_icon.setIconSize(QSize(48, 48))
        self.btn_animation_icon.setEnabled(True)
        self.btn_animation_icon.setStyleSheet("background-color: rgba(255, 255, 255, 0)")
        self.btn_animation_icon.setCursor(QtCore.Qt.PointingHandCursor)
        self.horizontalLayout_test_save.addWidget(self.btn_animation)
        self.horizontalLayout_test_save.addWidget(self.btn_animation_icon)

        # save file button
        self.btn_savefile = QLabel(self.test_save_widget)
        self.btn_savefile.setStyleSheet(
            "background-color: rgba(255, 255, 255, 0);font-size:30px;font-weight:bold;font-family:Times New Roman;")
        self.btn_savefile.setText("save joint")
        self.btn_savefile.setAlignment(Qt.AlignCenter)
        self.btn_savefile_icon = QPushButton(self.test_save_widget)
        self.btn_savefile_icon.setIcon(QIcon('icon/save.png'))
        self.btn_savefile_icon.setIconSize(QSize(48, 48))
        self.btn_savefile_icon.setEnabled(False)
        self.btn_savefile_icon.setStyleSheet("background-color: rgba(255, 255, 255, 0)")
        self.btn_savefile_icon.setCursor(QtCore.Qt.PointingHandCursor)
        self.horizontalLayout_test_save.addWidget(self.btn_savefile_icon)
        self.horizontalLayout_test_save.addWidget(self.btn_savefile)

        # joint type selection button
        self.joint_type_box = RadioBox(botton_names=("prismatic", "revolute"), size=(400, 100), mainwindow=self)

        # joint limit QlineEdit
        self.joint_limit_layout = QHBoxLayout()
        self.joint_lower_btn = QPushButton()
        self.joint_lower_btn.setText('lower')
        self.joint_upper_btn = QPushButton()
        self.joint_upper_btn.setText('upper')
        self.joint_lower = QLineEdit()
        self.joint_upper = QLineEdit()
        self.joint_limit_layout.addWidget(self.joint_lower_btn)
        self.joint_limit_layout.addWidget(self.joint_lower)
        self.joint_limit_layout.addWidget(self.joint_upper_btn)
        self.joint_limit_layout.addWidget(self.joint_upper)

        # self.joint_lower.setEchoMode(QLineEdit.Normal)
        # self.joint_upper.setEchoMode(QLineEdit.Normal)
        self.joint_limit_box = QWidget(self.tools_widget)
        self.joint_limit_box.setStyleSheet(
            "font-size:25px;font-weight:bold;font-family:Times New Roman;")
        self.joint_limit_box.setLayout(self.joint_limit_layout)
        self.joint_limit_box.setFixedSize(380, 80)

        # joint parent node QlineEdit
        self.joint_parent_node_layout = QHBoxLayout()
        self.joint_parent_btn = QPushButton()
        self.joint_parent_btn.setText('parent link')
        self.joint_parent_line = QLineEdit()
        self.joint_parent_node_layout.addWidget(self.joint_parent_btn)
        self.joint_parent_node_layout.addWidget(self.joint_parent_line)

        self.joint_parent_node_box = QWidget(self.tools_widget)
        self.joint_parent_node_box.setStyleSheet(
            "font-size:25px;font-weight:bold;font-family:Times New Roman;")
        self.joint_parent_node_box.setLayout(self.joint_parent_node_layout)
        self.joint_parent_node_box.setFixedSize(400, 80)

        # joint child node QlineEdit
        self.joint_child_node_layout = QHBoxLayout()
        self.joint_child_btn = QPushButton()
        self.joint_child_btn.setText('child link')
        self.joint_child_line = QLineEdit()
        self.joint_child_node_layout.addWidget(self.joint_child_btn)
        self.joint_child_node_layout.addWidget(self.joint_child_line)
        self.joint_child_node_box = QWidget(self.tools_widget)
        self.joint_child_node_box.setStyleSheet(
            "font-size:25px;font-weight:bold;font-family:Times New Roman;")
        self.joint_child_node_box.setLayout(self.joint_child_node_layout)
        self.joint_child_node_box.setFixedSize(400, 80)

        # annotation procedure button
        self.procedure_widget = QWidget(self)
        self.procedure_widget.setGeometry(QtCore.QRect(500, 250, 1200, 200))
        self.procedure_layout = QHBoxLayout(self.procedure_widget)
        self.annotation_procedure_box = ProcedureRadioBox(botton_names=("Axis Align", "Part Segmentation", "Joint Annotation"),
                                                          size=(1200, 200),
                                                          mainwindow=self)
        self.procedure_layout.addWidget(self.annotation_procedure_box)

        # reset button
        self.reset_btn = QPushButton(self.procedure_widget)
        self.reset_btn.setIcon(QIcon('icon/reset.png'))
        self.reset_btn.setIconSize(QSize(32, 32))
        self.reset_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.reset_btn.clicked.connect(self.on_btn_reset_clicked)
        self.procedure_layout.addWidget(self.reset_btn)


        #self.spacerItem1 = QtWidgets.QSpacerItem(80, 120, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.tools_layout.addWidget(self.articulation_tree)
        self.tools_layout.addWidget(self.test_save_widget)
        self.tools_layout.addWidget(self.joint_type_box)
        self.tools_layout.addWidget(self.joint_limit_box)
        self.tools_layout.addWidget(self.joint_parent_node_box)
        self.tools_layout.addWidget(self.joint_child_node_box)
        # self.tools_layout.addWidget(self.nxt_widget)

        self.btn_animation_icon.clicked.connect(self.on_btn_animation_clicked)
        self.btn_savefile_icon.clicked.connect(self.on_btn_record_joint_clicked)
        self.joint_lower_btn.clicked.connect(self.on_btn_joint_lower_clicked)
        self.joint_upper_btn.clicked.connect(self.on_btn_joint_upper_clicked)
        self.joint_parent_btn.clicked.connect(self.on_btn_joint_parent_clicked)
        self.joint_child_btn.clicked.connect(self.on_btn_joint_child_clicked)

        self.annotator_window = Annotator(annotation_material_path='annotation_materials/model_materials/')
        # annotator_window.annotate()

        self.label = AnnotationLabel(self.layout_widget, self.annotator_window, mainwindow=self)
        self.label.setFixedSize(1300, 900)
        self.label.move(450, 300)

        #self.listwidget_img.setStyleSheet('border-width: 5px;border-style: solid;border-color: rgb(255, 0, 0);')

        # self.setGeometry(300,300,300,200)
        self.setWindowTitle(__appname__)
        self.show()

        self.scrollAreaWidgetContents.find_widget_pos(self.gridLayout)

        self.file_loaded = False

    def selectionchange(self):
        pass
        self.current_selection = self.combo.currentText()
        self.annotator_window = Annotator(model_repository_path='resources/' + self.current_selection + '/pcmodels_articulation')
        for i in range(self.gridLayout.count()):
            self.gridLayout.itemAt(i).widget().deleteLater()
        self.start_img_viewer()

    def show_img(self, demo_img):
        # img = (np.asarray(self.annotator_window.demo_img_init) * 255).astype(np.uint8)
        img = (np.asarray(demo_img) * 255).astype(np.uint8)
        img_pil = Image.fromarray(img)
        # size = img_pil.size
        # self.label.setPixmap(img_pil.toqpixmap().scaled(size[0], size[1]))
        self.reshape_showing_img(img_pil, img.shape)

    def reshape_showing_img(self, img_pil, shape):
        if np.argmax(shape[:2]) == 1:
            image = img_pil.toqpixmap().scaled(self.label.width(), self.label.width() * shape[0] / shape[1])
        elif np.argmax(shape[:2]) == 0:
            image = img_pil.toqpixmap().scaled(self.label.width() * shape[1] / shape[0], self.label.width())

        self.label.setPixmap(image)
        # self.scale = (shape[0] / image.height(), shape[1] / image.width())
        self.scale = ((shape[0] / image.height()) * (image.width() / ((image.width() + self.label.width()) / 2)),
                      (shape[1] / image.width()) * (image.height() / ((image.height() + self.label.height()) / 2)))

    def start_img_viewer(self):
        initial_path = 'annotation_materials/thumbnails'
        photo_list = [os.path.join(initial_path, photo) for photo in os.listdir(initial_path)]
        # if os.path.splitext(photo)[-1].lower() == '.jpg']

        photo_num = len(photo_list)
        if photo_num != 0:
            for i in range(photo_num):
                image_id = photo_list[i]
                pixmap = QPixmap(image_id)
                self.addImage(pixmap, image_id)
                QApplication.processEvents()

    def addImage(self, pixmap, image_id):
        self.col += 1

        clickable_image = ClickableImage(self.display_image_size, self.display_image_size, pixmap, image_id,
                                         mainwindow=self)
        # clickable_image.clicked.connect(self.on_left_clicked)
        self.gridLayout.addWidget(clickable_image, 0, self.col)

    def import_model(self):
        self.model_path = QFileDialog.getExistingDirectory()
        self.temp_path = os.path.join(self.model_path, 'temp')
        if self.model_path != '':
            if not os.path.lexists(self.temp_path):
                os.makedirs(self.temp_path)
            self.annotator_window.temp_path = self.temp_path
            self.init_annotator()
            self.add_tree_item(self.model_path, parent='root')

    def add_tree_item(self, node_name, parent='root'):
        node_name = os.path.basename(node_name)
        if parent == 'root':
            # for object name root node
            self.object_name = node_name
            self.item_root = QTreeWidgetItem(self.articulation_tree)
            self.item_root.setText(0, self.object_name)
            # self.articulation_item_dict['root'] = {self.object_name: {}}
            self.articulation_item_list.append(self.item_root)
        elif parent == '':
            if not any(node_name == v.text(0) for v in self.articulation_item_list):
                # if the node already exists, continue; else add as root's child
                item_add = QTreeWidgetItem()
                item_add.setText(0, node_name)
                self.articulation_item_list[0].addChild(item_add)
                self.articulation_item_list.append(item_add)
        else:
            if not any(node_name == v.text(0) for v in self.articulation_item_list):
                for item in self.articulation_item_list:
                    if parent == item.text(0):
                        item_add = QTreeWidgetItem()
                        item_add.setText(0, node_name)
                        item.addChild(item_add)
                        self.articulation_item_list.append(item_add)

    def right_click_tree(self, pos):
        item = self.articulation_tree.currentItem()
        item1 = self.articulation_tree.itemAt(pos)
        if item != None and item1 != None:
            popMenu = QMenu()
            popMenu.addAction(QAction(u'joint info', self))
            # popMenu.addAction(QAction(u'bbb', self))
            popMenu.triggered[QAction].connect(self.on_tree_action_clicked)
            popMenu.exec_(QCursor.pos())

    def on_tree_action_clicked(self, q):
        command = q.text()
        item = self.articulation_tree.currentItem()
        joint_info_text = ''
        if command == 'joint info':
            for joint_info in self.annotator_window.annotated_joint_infos:
                if joint_info['child'].split('.')[0] == item.text(0):
                    joint_info_text = 'parent: {}\n' \
                                      'child: {}\n' \
                                      'xyz: {}\n' \
                                      'axis: {}\n' \
                                      'limit lower: {}\n' \
                                      'limit upper: {}\n'.format(joint_info['parent'], joint_info['child'],
                                                                 joint_info['xyz'], joint_info['rpy'],
                                                                 joint_info['lower'], joint_info['upper'])

            QMessageBox.information(self, "Information", joint_info_text, QMessageBox.Ok)

    def on_btn_back_clicked(self):
        self.choose_pre_nxt_image(choose_type='pre')

    def on_btn_nxt_clicked(self):
        self.choose_pre_nxt_image(choose_type='nxt')

    def on_btn_animation_clicked(self):
        parent_node_file = self.joint_parent_line.text()
        if parent_node_file == '':
            QMessageBox.warning(self, 'warning', 'please select parent mesh file!')
            return
        child_node_file = self.joint_child_line.text()
        if child_node_file == '':
            QMessageBox.warning(self, 'warning', 'please select child mesh file!')
            return
        lower = self.joint_lower.text()
        if lower == '':
            QMessageBox.warning(self, 'warning', 'please input the lower joint limit!')
            return
        upper = self.joint_upper.text()
        if upper == '':
            QMessageBox.warning(self, 'warning', 'please input the upper joint limit!')
            return
        self.annotator_window.set_animation_info(parent_node_file,
                                                 child_node_file,
                                                 lower,
                                                 upper,
                                                 self.joint_type_box.current_joint_type)

        self.annotator_window.play_animation()

    def on_btn_record_joint_clicked(self):
        parent_node_file = self.joint_parent_line.text()
        child_node_file = self.joint_child_line.text()
        lower = self.joint_lower.text()
        upper = self.joint_upper.text()

        self.annotator_window.record_joint_info(parent_node_file,
                                                child_node_file,
                                                lower,
                                                upper,
                                                self.joint_type_box.current_joint_type)
        parent_name = os.path.basename(parent_node_file).split('.')[0]
        child_name = os.path.basename(child_node_file).split('.')[0]
        self.add_tree_item(parent_name, parent='')
        self.add_tree_item(child_name, parent=parent_name)
        QMessageBox.information(self,
                                "information",
                                "joint info connecting parent: {} and child: {} has been correctly recorded"
                                .format(os.path.basename(parent_node_file), os.path.basename(child_node_file)))

    def on_btn_joint_lower_clicked(self):
        # a = QInputDialog()
        # a.setDoubleStep(0.1)
        # joint_limit, flag = a.getDouble(self, 'lower', 'input', 0.0, -np.inf, np.inf, 1)
        joint_limit, flag = QInputDialog.getDouble(self, 'lower', 'input', 0.0, -np.inf, np.inf, 1)

        if flag:
            self.joint_lower.setText(str(joint_limit))

    def on_btn_joint_upper_clicked(self):
        joint_limit, flag = QInputDialog.getDouble(self, 'upper', 'input', 0.0, -np.inf, np.inf, 1)

        if flag:
            self.joint_upper.setText(str(joint_limit))

    def on_btn_joint_parent_clicked(self):
        get_filename_path, flag = QFileDialog.getOpenFileName(self,
                                                              "Open Mesh File",
                                                              ".",
                                                              "Mesh Files (*.obj *.ply);;All Files (*)")
        if flag:
            self.joint_parent_line.setText(str(get_filename_path))
            self.check_save_joint()

    def on_btn_joint_child_clicked(self):
        get_filename_path, ok = QFileDialog.getOpenFileName(self,
                                                            "Open Mesh File",
                                                            ".",
                                                            "Mesh Files (*.obj *.ply);;All Files (*)")
        if ok:
            self.joint_child_line.setText(str(get_filename_path))
            self.check_save_joint()

    def on_btn_reset_clicked(self):
        rec_code = QMessageBox.information(self,
                                           'reset annotation',
                                           'Are you sure to reset the annotation? \n'
                                           'this will delete all the annotated info',
                                           QMessageBox.Yes | QMessageBox.No)

        # select Yes
        if rec_code != 65536:
            if not hasattr(self, 'temp_path'):
                QMessageBox.information(self,
                                        'Information',
                                        'there is no model to be annotated loaded \n'
                                        'please init annotator first!')
            else:
                for file in os.listdir(self.temp_path):
                    os.remove(os.path.join(self.temp_path, file))
                QMessageBox.information(self,
                                        'Information',
                                        'Annotation has been reset! \n'
                                        'now you can re-annotate the model')

    def init_annotator(self):
        # load annotator
        self.scale = (1, 1)
        self.annotator_window.model_to_be_annotated_path = self.model_path
        self.annotator_window.init_annotator()

        self.show_img(self.annotator_window.demo_img_init)
        self.label.update_scale(self.scale)
        self.label.grabKeyboard()

        self.file_loaded = True

    def begin_axis_align(self):
        if self.annotator_window.model_to_be_annotated is None:
            QMessageBox.warning(self, 'warning', 'load raw scanned model first!')
            return
        if self.annotator_window.annotation_material is None:
            QMessageBox.warning(self, 'warning', 'select annotation material first!')
            return
        print("start axis align")
        self.annotator_window.begin_annotation(stage="axis align")
        self.show_img(self.annotator_window.demo_img_axis_align)
        self.check_animation()

    def begin_part_segmentation(self):
        if self.annotator_window.model_to_be_annotated is None:
            QMessageBox.warning(self, 'warning', 'load raw scanned model first!')
        print("start part_segmentation")
        self.annotator_window.begin_annotation(stage="part segmentation")
        self.show_img(self.annotator_window.demo_img_part_segmentation)
        self.check_animation()

    def begin_joint_annotation(self):
        if self.annotator_window.model_to_be_annotated is None:
            QMessageBox.warning(self, 'warning', 'load raw scanned model first!')
        print("start joint_annotation")
        self.annotator_window.begin_annotation(stage="joint annotation")
        self.show_img(self.annotator_window.demo_img_joint_annotation)
        self.check_animation()

    def check_animation(self):
        if all(v is not None for v in [self.annotator_window.demo_img_axis_align,
                                       self.annotator_window.demo_img_part_segmentation,
                                       self.annotator_window.demo_img_joint_annotation]):
            self.btn_animation_icon.setEnabled(True)

    def check_save_joint(self):
        if all(v != '' for v in [self.joint_parent_line.text(),
                                 self.joint_child_line.text()]):

            self.btn_savefile_icon.setEnabled(True)

    def show_instruction(self):
        help_text = 'Keyboard Hotkey\n' \
                    '1: Rotation around roll axis positively\n' \
                    '2: Rotation around roll axis negatively\n' \
                    '3: Rotation around pitch axis positively\n' \
                    '4: Rotation around pitch axis negatively\n' \
                    '5: Rotation around yaw axis positively\n' \
                    '6: Rotation around yaw axis negatively\n' \
                    '7: Translation along prismatic joint positively\n' \
                    '8: Translation along prismatic joint negatively\n' \
                    '9: Rotation along revolute joint positively\n' \
                    '0: Rotation along revolute joint negatively\n' \

        QMessageBox.information(self, 'Instruction', help_text, QMessageBox.Ok)
        return

    def export_urdf(self):
        export_file_path, flag = QFileDialog.getSaveFileName(self,
                                                             "Export URDF",
                                                             ".",
                                                             "URDF Files (*.urdf)")
        if flag:
            self.annotator_window.save_urdf(export_file_path)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    demo1 = MainWindow()

    sys.exit(app.exec_())
