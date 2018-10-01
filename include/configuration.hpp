/**
 * Implements the RASM's configuration subsystem.
 * Defines the ConfigGroup and ConfigurationManager (singleton) classes.
 */

#ifndef CONFIGURATION_INCLUDED
#define CONFIGURATION_INCLUDED

#include <unordered_set>
#include <Poco/Util/MapConfiguration.h>
#include <Poco/Util/PropertyFileConfiguration.h>
#include <Poco/String.h>
#include <Poco/Clock.h>
#include <Poco/Util/Timer.h>
#include <Poco/Util/TimerTask.h>
#include <Poco/AutoPtr.h>

using std::string;
using Poco::Util::MapConfiguration;
using Poco::Util::PropertyFileConfiguration;

/**
 * For internal use by the ConfigurationManager class.
 * Holds all java-style configurations from a single .properties file given
 * by the filepath argument. Any configuration can have it's values retreived,
 * changed, or saved back to the file.
 */
class ConfigGroup
{
public:
  PropertyFileConfiguration *file_config_map;
  string filepath;
  bool modified;

  ConfigGroup(string filepath)
  {
    file_config_map = new PropertyFileConfiguration(filepath);
    this->filepath = filepath;
    modified = false;
  }

  ConfigGroup(ConfigGroup &other)
  {
    file_config_map = other.file_config_map;
    filepath = other.filepath;
    modified = other.modified;
  }

  void operator=(ConfigGroup &other)
  {
    file_config_map = other.file_config_map;
    filepath = other.filepath;
    modified = other.modified;
  }

  /**
   * Sets value to the configuration value that corresponds to the given
   * key if that key exists. If the key doesn't exist then value is set to
   * an empty string. Returns true if the key exists; false otherwise.
   * Note that keys are case sensitive.
   */
  bool get_value(const string &key, string &value)
  {
    if (file_config_map->hasProperty(key))
    {
      value = file_config_map->getString(key);
      return true;
    }
    else
    {
      value = "";
      return false;
    }
  }

  /**
   * Changes the old value corresponding to the given key to the given value
   * if the key exists. Returns true if the key exists (and subsequently the
   * value change was made); false otherwise.
   * Note that keys are case sensitive.
   */
  bool change_value(const string &key, const string &value)
  {
    if (file_config_map->hasProperty(key))
    {
      file_config_map->setString(key, value);
      modified = true;
      return true;
    }
    return false;
  }

  /**
   * Saves the current PropertyFileConfiguration instance to the file given
   * by the stored filepath if any calls to change_configuration have been
   * made since construction or since the last call to this method.
   */
  void push_to_file()
  {
    if (modified)
    {
      file_config_map->save(filepath);
      modified = false;
    }
  }
};

/**
 * This class is a singleton that manages a collection of ConfigGroup
 * instances. There is one instance for each of the following enum-file
 * correspondences:
 *  Enum              File
 *  - - - - - - - -   - - - - - - - - - - - - - - - - -
 *  GENERAL           <rwd>/config/general.properties
 *  LOGGING           <rwd>/config/logging.properties
 *  BATTERY           <rwd>/config/battery.properties
 *  POSE_ESTIMATION   <rwd>/config/pose_estimation.properties
 *  CONTROL           <rwd>/config/control.properties
 *  PERIPHERY         <rwd>/config/periphery.properties
 *  HTTP_SERVER       <rwd>/config/http_server.properties
 *  SHELL_SERVER      <rwd>/config/shell_server.properties
 *
 * rwd stands for root working directory and exists as a public static field
 * which must be set prior to this singleton's construction (happens when
 * obtaining the singleton instance for the first time).
 */
class ConfigurationManager : public Poco::Util::TimerTask
{
public:
  ConfigurationManager(const ConfigurationManager &) = delete;
  void operator=(const ConfigurationManager &) = delete;

  static char *rwd;  // the root working directory - set prior to construction

  /**
   * An enumerator representation of each configuration group/file managed
   * by this configuration manager.
   */
  enum Group
  {
    GENERAL,
    LOGGING,
    BATTERY,
    POSE_ESTIMATION,
    CONTROL,
    PERIPHERY,
    HTTP_SERVER,
    SHELL_SERVER
  };

private:
  string root_working_dir;
  std::map<Group, ConfigGroup*> config_groups;
  std::unordered_set<string> modifiables;  // keys of modifiable values
  Poco::Util::Timer sync_timer;  // filesystem synchronization timer
  Poco::Clock::ClockDiff sync_interval;  // filesystem syncronization interval

  ConfigurationManager()
  {
    root_working_dir = string(rwd);
    if (root_working_dir == "")
      throw std::runtime_error("the root working directory hasn't been set");
    string configroot = root_working_dir + "/config";

    // load all properties from each configuration file (one .properties file per group)
    config_groups[Group::GENERAL] = new ConfigGroup(configroot + "/general.properties");
    config_groups[Group::LOGGING] = new ConfigGroup(configroot + "/logging.properties");
    config_groups[Group::BATTERY] = new ConfigGroup(configroot + "/battery.properties");
    config_groups[Group::POSE_ESTIMATION] = new ConfigGroup(configroot + "/pose_estimation.properties");
    config_groups[Group::CONTROL] = new ConfigGroup(configroot + "/control.properties");
    config_groups[Group::PERIPHERY] = new ConfigGroup(configroot + "/periphery.properties");
    config_groups[Group::HTTP_SERVER] = new ConfigGroup(configroot + "/http_server.properties");
    config_groups[Group::SHELL_SERVER] = new ConfigGroup(configroot + "/shell_server.properties");

    // specify the keys whos values are modifiable
    // all keys should be lowercase
    //modifiables.emplace("key1");
    //modifiables.emplace("key2");

    // start the filesystem synchronization timer
    sync_interval = 60 * 1000 * 1000;  // 1 minute in microseconds
    reschedule_sync_timer();
  }

  /**
   * Synchronization timer callback method (required via extension of TimerTask).
   */
  void run()
  {
    save_config_changes();
  }

  /**
   * Cancels the currently running synchronization timer and reschedules it
   * to run at sync_interval microseconds in the future.
   */
  void reschedule_sync_timer()
  {
    Poco::Clock currentTime;
    sync_timer.cancel(false);
    sync_timer.schedule(this, currentTime + sync_interval);
  }

public:
  /**
   * Returns this singleton's instance.
   */
  static ConfigurationManager& get_instance()
  {
    static ConfigurationManager instance;
    return instance;
  }

  /**
   * Returns a pointer to a MapConfiguration instance that holds a copy of
   * all configurations for a particular configuration group. Note that the
   * configurations may not match what is currently contained in the
   * configuration files, but will be the most up-to-date settings.
   */
  Poco::AutoPtr<MapConfiguration> get_config_group(Group group)
  {
    Poco::AutoPtr<MapConfiguration> mapCopy(new MapConfiguration());
    config_groups[group]->file_config_map->copyTo(*mapCopy);
    mapCopy->setString("root_working_dir", root_working_dir);
    return mapCopy;
  }

  /**
   * Sets value to the configuration value that corresponds to the given
   * key if that key exists within the given configuration group. If the key
   * doesn't exist then value is set to an empty string. Returns true if the
   * key exists; false otherwise.
   */
  bool get_config_value(Group group, string key, string &value)
  {
    // convert key to all lowercase with no leading/trailing whitespace
    Poco::trimInPlace(key);
    Poco::toLowerInPlace(key);
    return config_groups[group]->get_value(key, value);
  }

  /**
   * Overwrites the current value corresponding to the key in the specified
   * configuration group if the key exists and has a value that is
   * modifiable. If the value is successfully modified then true is returned,
   * otherwise false is returned.
   */
  bool change_config_value(Group group, string key, const string &value)
  {
    // convert key to all lowercase with no leading/trailing whitespace
    Poco::trimInPlace(key);
    Poco::toLowerInPlace(key);

    // exit if the value of the key isn't allowed to be modified
    if (modifiables.count(key) == 0)
      return false;

    return config_groups[group]->change_value(key, value);
  }

  /**
   * Overwrites certain configuration files in order to reflect the changes
   * in the configuration groups that have had modifications via the
   * change_config_value method.
   */
  void save_config_changes()
  {
    for (auto &pair : config_groups)
      pair.second->push_to_file();
    reschedule_sync_timer();
  }
};

#endif
