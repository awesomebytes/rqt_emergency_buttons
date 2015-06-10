
from python_qt_binding.QtCore import QSize

from rqt_robot_dashboard.widgets import IconToolButton
from .icon_widget import IconWidget


class EmergencyButton(IconWidget):
    """
    Dashboard widget to display emergency button state.
    """
    def __init__(self, context, name):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        icon_names = ['emergency_button_gray.png','emergency_button_color.png']
        icons = []
        for icon in icon_names:
            icons.append([icon])
        icon_paths = []
        icon_paths.append(['rqt_emergency_buttons', 'images'])
        self._name = name
        super(EmergencyButton, self).__init__(name=name,
                                                  icons=icons,
                                                  icon_paths=icon_paths,
                                                  suppress_overlays=True)
        self.setToolTip(name)
        self.set_non_pressed()

    def set_non_pressed(self):
        self.update_state(0)
        self.setToolTip(self._name + " is NOT pressed.")

    def set_pressed(self):
        self.update_state(1)
        self.setToolTip(self._name + " IS pressed.")

    def set_pressed_status(self, pressed):
        if pressed:
            self.set_pressed()
        else:
            self.set_non_pressed()
