# Add a new Plugin

In order to implement a new plugin the followings steps need to be done:
1. Create a new Class "\<Pluginname\>Plugin" as a plugin using the ROS Pluginlib (http://wiki.ros.org/pluginlib)
    1. Locate it e.g. in the existing namespace "hector_joy_teleop_plugins" or in a new namespace.
    2. Make it an derived class of class "hector_joy_teleop_plugin_interface::TeleopBase".
    3. Overwrite the initialize method and inside of it first call the method 
    `TeleopBase::initializeBase(nh, pnh, "<namespace>::<pluginname>");`.
    4. Overwrite / Implement the function  
    `void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;` with the plugin functionality.
    5. If required, implement the function `executePeriodically(const ros::Rate& rate)` for parts that should be executed
    in every run of the ros loop.
2. According to the usage of the ros pluginlib add the following line at the bottom of the source file of the new plugin:
    `PLUGINLIB_EXPORT_CLASS(<namespace>::<classname>, hector_joy_teleop_plugin_interface::TeleopBase)`
3. Also according to the usage of the ros pluginlib add the following lines for the new plugin to a xml file containing the library:
    ```xml
   <class type="namespace::classname" base_class_type="hector_joy_teleop_plugin_interface::TeleopBase">
        <description>Plugindescription</description>  
    </class>
4. Add configuration files:
    1. A mapping file containing the axes and button mapping. (e.g. use empty_mapping_logitech_f710.yaml as template,
    so that it is parsed correctly by PluginBase.)
    2. If required add a common configuration file for the plugin containing other definitions e.g. max values. (In order to get the so defined parameters from parameterserver use 
    `pnh_.param(getParameterServerPrefix() + "/" + "parametername", default);`)
5. Add plugin to launch file of hector_joy_with_plugins:
    1. Add the configuration files to the node in its own namespace e.g. using
    `<rosparam file="$(find <package containing plugin>)/config/<plugin-(short-)name>/<plugin-(short-)name>_mapping_logitech_f710.yaml" ns="<namespace of plugin>/<Pluginname>" />`
    2. If the plugin shall be loaded on startup add it to the list of the rosparam "init_plugins" with `<namespace>::<Pluginname>`.