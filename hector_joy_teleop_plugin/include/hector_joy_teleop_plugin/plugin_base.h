#pragma once

#include <map>
#include <string>


class PluginBase
{
 public:
  PluginBase();

  bool isActive();

  bool setActive(bool active);

  std::string getPluginName();

  /**
   * get button mappings of plugin
   * @return first: axes map, second: buttons map
   */
  std::pair<std::map<std::string, int>&,std::map<std::string, int>&> getMapping();

  /**
   * Convert and forward joy message to respective topic/package. Details in each plugin.
   */
  virtual void forwardMsg() = 0;

 private:
  bool active_ = false;

  // mapping of usage names to message array index
  std::map<std::string, int> axes_;
  std::map<std::string, int> buttons_;

  std::string plugin_name_;

};
