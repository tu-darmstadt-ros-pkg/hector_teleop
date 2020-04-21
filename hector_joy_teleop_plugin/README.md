# Load/unload a Plugin

Plugins are loaded/unloaded using the rosservice "/hector_joy_teleop_plugin/LoadPlugin".

The service has the following parameters:
* **Request**:
    * **string plugin_name**: Name of the plugin which should be (un-)loaded.
    * **bool load**: true to load the plugin, false to unload the plugin.

* **Response**:
    * **int8 result**: Result of the loading/unloading with the constants below.
    * **string overlapping_plugin**: If there was a plugin whose axes/buttons mapping overlaps with the current one, its name is given here.

* **Result constants**:
    * int8 SUCCESS=0
    * int8 UNKNOWN_PLUGINNAME=1
    * int8 OVERLAPPING_BUTTON_MAPPING=2



# Add a new Plugin

In order to implement a new plugin the followings steps need to be done:
1. Create a new Class "\<Pluginname\>Plugin" in directory .../plugins/.
    1. Make it an derived class of class "PluginBase".
    2. In constructor call the super constructor to ensure the correct parsing of the button mapping.
    3. Overwrite/Implement the function  
    `void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;`
    4. Implement the plugin functionality.
2. Add a new case for the new plugin in JoyTeleopPlugin::pluginFactory.
3. Add configuration files:
    1. A mapping file containing the axes and button mapping. (e.g. use empty_mapping_logitech_f710.yaml as template,
    so that it is parsed correctly by PluginBase.)
    2. If required add a common configuration file for the plugin containing other definitions e.g. max values.
4. Add plugin to launch file:
    1. Add the configuration files to the node in its own namespace e.g. using
    `<rosparam file="$(find hector_joy_teleop_plugin)/config/plugins/<Pluginname>_mapping_logitech_f710.yaml" ns="<Pluginname>Plugin" />`
    2. If the plugin shall be loaded on startup add it to the list of the rosparam "init_plugins".