
import os
import rospkg
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QIcon, QLabel
from rqt_robot_dashboard.util import IconHelper

class IconWidget(QLabel):
    """
    A Widget which displays some state, non clickable, including a status tip.

    :param name: The name of this widget
    :type name: str
    """
    state_changed = Signal(int)

    def __init__(self, name='Icon', icons=None, icon_paths=None, suppress_overlays=False):
        super(IconWidget, self).__init__()
        if icons == None: # Give some defaults...
            icons = []
            icons.append(['bg-red.svg'])
            icons.append(['bg-green.svg'])
        icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))
        self._icon_helper = IconHelper(paths, name)
        converted_icons = self._icon_helper.set_icon_lists(icons, suppress_overlays=suppress_overlays)
        self._icons = converted_icons[0]
        self._name = name
        self.__state = 0
        self.setMargin(5)
        self.state_changed.connect(self._update_state)

    def _update_state(self, state):
        self.setPixmap(self._icons[state].pixmap(QSize(60, 100)))

    @property
    def state(self):
        """
        Read-only accessor for the widgets current state.
        """
        return self.__state

    def update_state(self, state):
        """
        Set the state of this button.
        This will also update the icon for the button based on the ``self._icons`` list

        :raises IndexError: If state is not a proper index to ``self._icons``

        :param state: The state to set.
        :type state: int
        """
        if 0 <= state and state < len(self._icons):
            self.__state = state
            self.state_changed.emit(self.__state)
        else:
            raise IndexError("%s update_state received invalid state: %s" % (self._name, state))
