/**
 * Implements the RASM's configuration subsystem.
 * Defines the ConfigGroup, ConfigurationManagerImpl, and ConfigurationManager
 * (singleton) classes.
 */

#ifndef RASM2_CONFIGURATION_HPP
#define RASM2_CONFIGURATION_HPP

#include <Poco/Util/MapConfiguration.h>
#include <Poco/Util/PropertyFileConfiguration.h>
#include <Poco/String.h>
#include <Poco/Clock.h>
#include <Poco/Util/Timer.h>
#include <Poco/Util/TimerTask.h>
#include <Poco/File.h>
#include <Poco/Path.h>

using std::string;
using Poco::Util::MapConfiguration;
using Poco::Util::PropertyFileConfiguration;

/**
 * For internal use by the ConfigurationManager class.
 * Holds all java-style configurations from a single .conf file given by the
 * filepath argument. Any configuration can have it's values retreived,
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

  ~ConfigGroup()
  {
    file_config_map->release();
  }

  /**
   * Sets value to the configuration value that corresponds to the given
   * key if that key exists. If the key doesn't exist then value is set to
   * an empty string. Returns true if the key exists; false otherwise.
   * Note that keys are case sensitive.
   */
  bool get_value(const string &key, string &value) const
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

  /**
   * Equals assignment operator.
   */
  ConfigGroup & operator=(const ConfigGroup &other)
  {
    if (*this == other)
      return *this;

    file_config_map = other.file_config_map;
    filepath = other.filepath;
    modified = other.modified;
    return *this;
  }

  /**
   * Equals comparison operator.
   */
  bool operator==(const ConfigGroup &other) const
  {
    return modified == other.modified &&
        filepath == other.filepath &&
        file_config_map == other.file_config_map;
  }

  /**
   * Not equals comparison operator.
   */
  bool operator!=(const ConfigGroup &other) const
  {
    return !operator==(other);
  }
};

/**
 * This class is a singleton that manages a collection of ConfigGroup
 * instances. There is one instance for each of the following enum-file
 * correspondences:
 *  Enum              File
 *  - - - - - - - -   - - - - - - - - - - - - - - - - -
 *  OTHER             rwd/config/other.conf
 *  LOGGING           rwd/config/logging.conf
 *  BATTERY           rwd/config/battery.conf
 *  VISION            rwd/config/vision.conf
 *  CONTROL           rwd/config/control.conf
 *  PERIPHERY         rwd/config/periphery.conf
 *  HTTP              rwd/config/http.conf
 *  SHELL             rwd/config/shell.conf
 *
 * rwd stands for root working directory and is set by the ${HOME}/.rasm.conf
 * file which should have the key value pair 'root_working_dir = ...'.
 */
class ConfigurationManagerImpl : public Poco::Util::TimerTask
{
public:
  ConfigurationManagerImpl(const ConfigurationManagerImpl &) = delete;
  void operator=(const ConfigurationManagerImpl &) = delete;

  /**
   * An enumerator representation of each configuration group/file managed
   * by this configuration manager.
   */
  enum Group
  {
    OTHER,
    LOGGING,
    BATTERY,
    VISION,
    CONTROL,
    PERIPHERY,
    HTTP,
    SHELL
  };

private:
  string root_working_dir;
  std::map<Group, std::shared_ptr<ConfigGroup> > config_groups;
  Poco::Util::Timer sync_timer;  // filesystem synchronization timer
  Poco::Clock::ClockDiff sync_interval;  // filesystem syncronization interval

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
  ConfigurationManagerImpl()
  {
    // get the root working directory from the $(HOME)/.rasm.conf file
    Poco::File startup_conf_file(Poco::Path::home() + ".rasm.conf");
    if (!startup_conf_file.exists())
      throw std::runtime_error("The configuration file " + startup_conf_file.path() + " doesn't exist.");
    PropertyFileConfiguration *startup_configs = new PropertyFileConfiguration(startup_conf_file.path());
    try {
      root_working_dir = startup_configs->getString("root_working_dir");
    } catch (Poco::NotFoundException) {
      throw std::runtime_error("The configuration file " + startup_conf_file.path() +
          " must define the 'root_working_dir' property");
    }

    // load all properties from each configuration file (one .conf file per group)
    string configroot = root_working_dir + "/config/";
    config_groups[Group::OTHER].reset(new ConfigGroup(configroot + "other.conf"));
    config_groups[Group::LOGGING].reset(new ConfigGroup(configroot + "logging.conf"));
    config_groups[Group::BATTERY].reset(new ConfigGroup(configroot + "battery.conf"));
    config_groups[Group::VISION].reset(new ConfigGroup(configroot + "vision.conf"));
    config_groups[Group::CONTROL].reset(new ConfigGroup(configroot + "control.conf"));
    config_groups[Group::PERIPHERY].reset(new ConfigGroup(configroot + "periphery.conf"));
    config_groups[Group::HTTP].reset(new ConfigGroup(configroot + "http.conf"));
    config_groups[Group::SHELL].reset(new ConfigGroup(configroot + "shell.conf"));

    // start the filesystem synchronization timer; default to 60 seconds if
    // period is not specified
    sync_interval = 1000 * 1000 * config_groups[Group::OTHER]->
        file_config_map->getInt("config.autosync.period_seconds", 60);
    reschedule_sync_timer();
  }

  /**
   * Returns a pointer to a MapConfiguration instance that holds a copy of
   * all configurations for a particular configuration group. Note that the
   * configurations may not match what is currently contained in the
   * configuration files, but will be the most up-to-date settings.
   */
  const MapConfiguration * get_config_group(Group group)
  {
    return config_groups[group]->file_config_map;
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

/**
 *
 */
class ConfigurationManager
{
private:
  ConfigurationManagerImpl impl;

  ConfigurationManager()
  {
  }

public:
  ConfigurationManager(const ConfigurationManagerImpl &) = delete;
  void operator=(const ConfigurationManagerImpl &) = delete;

  /**
   * Returns this singleton's instance.
   */
  static ConfigurationManager& get_instance()
  {
    static ConfigurationManager instance;
    return instance;
  }

  const MapConfiguration * get_config_group(ConfigurationManagerImpl::Group group)
  {
    return impl.get_config_group(group);
  }

  bool get_config_value(ConfigurationManagerImpl::Group group, string key, string &value)
  {
    return impl.get_config_value(group, key, value);
  }

  bool change_config_value(ConfigurationManagerImpl::Group group, string key, const string &value)
  {
    return impl.change_config_value(group, key, value);
  }

  void save_config_changes()
  {
    impl.save_config_changes();
  }
};

#endif
