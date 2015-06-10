import rospy

from rqt_robot_dashboard.dashboard import Dashboard

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from std_msgs.msg import Bool
from .emergency_button import EmergencyButton


class EmergencyButtonsDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Emergency Buttons Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._widget_initialized = False

        NAMESPACE = '/emergency_buttons_dashboard'
        if rospy.has_param(NAMESPACE):
            # rosparam set /emergency_buttons_dashboard/emergency_button "{'emergency1': {'pressed_topic': '/emergency1', 'tooltip_name': 'EMERGENCY1'}}"
            if rospy.has_param(NAMESPACE + '/emergency_buttons'):
                self._emergency_buttons_list = rospy.get_param(NAMESPACE + '/emergency_buttons')
            else:
                rospy.logwarn("No emergency buttons to monitor found in param server under " + NAMESPACE)
        else:
            rospy.logerr("You must set " + NAMESPACE +" parameters to use this plugin. e.g.:\n" +
                         "rosparam set "+ NAMESPACE + "/emergency_buttons \"{'emergency1': {'pressed_topic': '/emergency1', 'tooltip_name': 'EMERGENCY1'}}\"")
            exit(-1)

        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                pressed_topic = emergency_button_elem[emer_name].get('pressed_topic', None)
                tooltip_name = emergency_button_elem[emer_name].get('tooltip_name', None)
                rospy.loginfo("Emergency button: " + str(emer_name) + " has pressed topic: " +
                    str(pressed_topic)  + " and has tooltip name: " + str(tooltip_name))

                emergency_button_elem[emer_name].update({'pressed_status': False})
                emergency_button_elem[emer_name].update({'pressed_sub': rospy.Subscriber(pressed_topic,
                                                                                    Bool,
                                                                                    self.dashboard_callback,
                                                                                    callback_args={'emergency': emer_name},
                                                                                    queue_size=1)})

                emergency_button_elem[emer_name].update({'emergency_widget': EmergencyButton(self.context, name=tooltip_name)})

        self._widget_initialized = True


    def get_widgets(self):
        widgets_list = []
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                widgets_list.append([emergency_button_elem[emer_name]['emergency_widget']])

        return widgets_list

    def dashboard_callback(self, msg, cb_args):
        """
        callback to process messages

        :param msg:
        :type msg: Float32 or Bool
        :param cb_args:
        :type cb_args: dictionary
        """
        if not self._widget_initialized:
            return

        if cb_args.has_key('emergency'):
            emer_name = cb_args['emergency']
            for emergency_button_elem in self._emergency_buttons_list:
                if emergency_button_elem.has_key(emer_name):
                    emergency_button_elem[emer_name].update({'pressed_status': msg.data})

        # Throttling to 1Hz the update of the widget whatever the rate of the topics is
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0):
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update all widgets
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                emergency_button_elem[emer_name]['emergency_widget'].set_pressed_status(emergency_button_elem[emer_name]['pressed_status'])
                rospy.loginfo("Updated " + str(emer_name) + " with "
                              + ("pressed." if emergency_button_elem[emer_name].get('pressed_status') else "not pressed."))


    def shutdown_dashboard(self):
        for emergency_button_elem in self._emergency_buttons_list:
            for emer_name in emergency_button_elem.keys():
                if emergency_button_elem[emer_name]['pressed_sub']:
                    emergency_button_elem[emer_name]['pressed_sub'].unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # self._console.save_settings(plugin_settings, instance_settings)
        # self._monitor.save_settings(plugin_settings, instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # self._console.restore_settings(plugin_settings, instance_settings)
        # self._monitor.restore_settings(plugin_settings, instance_settings)
        pass
