# Load/unload a Plugin

Plugins are loaded/unloaded using the rosservice "/hector_joy_teleop_with_plugins/LoadPlugin".

The service has the following parameters:
* **Request**:
    * **string plugin_name**: Name of the plugin which should be (un-)loaded as `<namespace>::<Classname>`.
    * **bool load**: true to load the plugin, false to unload the plugin.

* **Response**:
    * **int8 result**: Result of the loading/unloading with the constants below.
    * **string error_msg**:
        * unknown plugin_name: given plugin_name
        * Overlapping button mapping: name of (already loaded) overlapping plugin.
        * error while calling plugins method "onLoad"/"onUnload": provided error message of plugin

* **Result constants**:
    * int8 SUCCESS=0
    * int8 UNKNOWN_PLUGINNAME=1
    * int8 OVERLAPPING_BUTTON_MAPPING=2
    * int8 PLUGIN_LOAD_ERROR=3
    
### Special requests:
* Unload all loaded plugins: set plugin_name to "all" and load to false.
