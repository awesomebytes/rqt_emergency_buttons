rqt plugin dashboard for showing emergency button status.

![Screenshot of the emergency buttons plugin working](https://raw.githubusercontent.com/pal-robotics/rqt_emergency_buttons/master/images/rqt_emergency_buttons_ss.png)

You must set in the param server
which topics to listen to and it will
do the magic for you.

The necessary parameters are:

    Topic(s) with Bool data showing if emergency button is pressed.

Example parameter server configuration can be found in the
 config folder. Looks like:

    emergency_buttons_dashboard:
        emergency_buttons:
            # Must be an array (it's parsed like that) so the "- " must be added. 
            # This is the name of the emergency button internally in the node, won't be shown
            - emergency1:
                # topic of type std_msgs/Bool publishing if the button is pressed (True)
                pressed_topic: /emergency1
                # name of the emergency button for the tooltip when mousing over
                tooltip_name: EMER1
            - emergency2:
                pressed_topic: /emergency2
                tooltip_name: EMER2
