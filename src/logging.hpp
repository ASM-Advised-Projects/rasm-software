/**
 * Implements the RASM's logging subsystem.
 * Defines the LogFile and LogManager (singleton) classes.
 */

#ifndef LOGGING_INCLUDED
#define LOGGING_INCLUDED

#include "configuration.hpp"

#include <fstream>
#include <sstream>
#include <sys/statvfs.h>

#include <Poco/Thread.h>
#include <Poco/Mutex.h>
#include <Poco/File.h>
#include <Poco/NumberParser.h>
#include <Poco/Clock.h>
#include <Poco/Util/TimerTask.h>
#include <Poco/Util/TimerTaskAdapter.h>

using std::string;


/**
 *
 */
class LogFile
{
public:
  string path;
  std::ofstream stream;

  LogFile(const LogFile &lf) = delete;
  void operator=(const LogFile &lf) = delete;

  LogFile()
  {
  }

  /**
   *
   */
  void init(string &filepath)
  {
    close();
    path = filepath;
    stream.open(path, std::ofstream::out | std::ofstream::app);
  }

  /**
   *
   */
  void close()
  {
    if (stream.is_open())
    {
      stream.flush();
      stream.close();
    }
  }

  /**
   *
   */
  unsigned long get_size() const
  {
    std::ifstream in(path, std::ofstream::ate | std::ofstream::binary);
    return in.tellg();
  }
};


/**
 *
 */
struct TimeStamp
{
  string HH;
  string MM;
  string SS;
  string LLL;
};


/**
 *
 */
class LogManager
{
public:
  LogManager(const LogManager &) = delete;
  void operator=(const LogManager &) = delete;

  /**
   * Returns this singleton's instance.
   */
  static LogManager& get_instance()
  {
    static LogManager instance;
    return instance;
  }

  /**
   * An enumerator representation of each log level recognized by this log
   * manager.
   */
  enum LogLevel
  {
    NOTICE,
    WARNING,
    ERROR,
    CRITICAL
  };

  /**
   * HH-MM-SS-LLL  <thread name>:<thread id>:<thread priority>
   * <message>\n\n
   */
  void log_message(const string &msg, LogLevel level)
  {
    Poco::Thread *current_thread = Poco::Thread::current();

    // get the current thread's name, id, and priority
    string thread_name;
    int thread_id;
    int thread_prio;
    if (current_thread == 0)  // main thread
    {
      thread_name = "main";
      thread_id = 0;
      thread_prio = 0;
    }
    else
    {
      thread_name = current_thread->getName();
      thread_id = current_thread->id();
      thread_prio = current_thread->getOSPriority();
    }

    // get the current time stamp
    TimeStamp ts = elapsed_time();

    // assemble the log message header
    std::ostringstream msg_builder;
    msg_builder << ts.HH << "-" << ts.MM << "-" << ts.SS << "-" << ts.LLL
        << "  " << thread_name << ":" << thread_id << ":" << thread_prio << "\n";
    msg_builder << msg << "\n\n";

    // log the header-ed message via switch case fall through
    log_mutex.lock();
    switch (level)
    {
      case CRITICAL:
        file_map[CRITICAL]->stream << msg_builder.str();
        file_map[CRITICAL]->stream.flush();
        rotate_log_check(CRITICAL);

      case ERROR:
        if (level != ERROR)
            file_map[ERROR]->stream << level_as_string(level, true) << "\n";
        file_map[ERROR]->stream << msg_builder.str();
        file_map[ERROR]->stream.flush();
        rotate_log_check(ERROR);

      case WARNING:
        if (level != WARNING)
            file_map[WARNING]->stream << level_as_string(level, true) << "\n";
        file_map[WARNING]->stream << msg_builder.str();

      case NOTICE:
        if (level != NOTICE)
            file_map[NOTICE]->stream << level_as_string(level, true) << "\n";
        file_map[NOTICE]->stream << msg_builder.str();
      }
      log_mutex.unlock();
  }

  /**
   * Logs the given exception's stack trace and message at the ERROR level.
   */
  void log_exception(const std::exception &exc)
  {

  }

private:
  std::map<LogLevel, LogFile*> file_map;
  Poco::Util::TimerTaskAdapter<LogManager> *flush_timer;
  string log_dir;
  Poco::Clock startup_time;
  long max_filesize;
  Poco::Mutex log_mutex;

  LogManager()
  {
    // get logging configuration group
    ConfigurationManager &configs = ConfigurationManager::get_instance();
    Poco::AutoPtr<MapConfiguration> log_conf =
        configs.get_config_group(ConfigurationManager::Group::LOGGING);

    // get max log file size
    max_filesize = 1000 * log_conf->getInt64("max_filesize_KB");

    // get directory names
    int dir_int = log_conf->getInt("next_log_num");
    int max_dir_int = log_conf->getInt("max_log_num");

    string root_dir = log_conf->getString("root_dir");
    string toplevel_dir = log_conf->getString("toplevel_dir");

    // wrap log directory number back to 1 if larger than max
    if (dir_int > max_dir_int)
      dir_int = 1;

    // create new log directory file object (don't create directory yet)
    log_dir = root_dir + toplevel_dir + "/" + std::to_string(dir_int);
    Poco::File new_log_dir(log_dir);

    // remove similarly named directory if it exists
    if (new_log_dir.exists())
      new_log_dir.remove(true);

    // create new log directory
    new_log_dir.createDirectory();

    // increment the next log directory's name in configurations
    configs.change_config_value(ConfigurationManager::Group::LOGGING,
        "next_log_dir", std::to_string(dir_int + 1));

    // ensure requisite filespace exists for log system by potentially
    // deleting old log directories
    struct statvfs partition_info;
    if (statvfs("/", &partition_info) == 0)
    {
      long bytes_to_remove = 1000 * log_conf->getInt("min_log_space_KB")
          - partition_info.f_bfree * partition_info.f_bsize;

      int count = 0;
      for (int oldest_dir_int = dir_int + 1; bytes_to_remove > 0; oldest_dir_int++)
      {
        if (oldest_dir_int > max_dir_int)
          oldest_dir_int = 1;

        std::string old_dir_path = root_dir + toplevel_dir + "/"
            + std::to_string(oldest_dir_int);
        Poco::File old_dir(old_dir_path);
        if (old_dir.isDirectory())
        {
          bytes_to_remove -= old_dir.getSize();
          old_dir.remove(true);
        }
      }
    }

    // map each log level to a LogFile instance
    file_map[NOTICE] = new LogFile();
    file_map[WARNING] = new LogFile();
    file_map[ERROR] = new LogFile();
    file_map[CRITICAL] = new LogFile();

    // initialize LogFile classes
    for (const auto &pair : file_map)
      rotate_log_check(pair.first);

    // start NOTICE and WARNING log buffer flush timers
    // error and critical are immediately written to file
    flush_timer = new Poco::Util::TimerTaskAdapter<LogManager>(
        *this, &LogManager::flush_notice_log);
    flush_timer = new Poco::Util::TimerTaskAdapter<LogManager>(
        *this, &LogManager::flush_warning_log);
  }

  ~LogManager()
  {
    for (auto &pair : file_map)
        delete pair.second;
    file_map.~map();
    log_dir.~basic_string();
    startup_time.~Clock();
  }

  void flush_notice_log(Poco::Util::TimerTask &)
  {
    log_mutex.lock();
    file_map[NOTICE]->stream.flush();
    rotate_log_check(NOTICE);
    log_mutex.unlock();
  }

  void flush_warning_log(Poco::Util::TimerTask &)
  {
    log_mutex.lock();
    file_map[WARNING]->stream.flush();
    rotate_log_check(WARNING);
    log_mutex.unlock();
  }

  /**
   * Rotates the log file for the given log level if it is nearing or
   * exceeding the max log file size. If a log file for the level doesn't
   * exist yet then a file will be created.
   */
  void rotate_log_check(const LogLevel &level)
  {
    // if the file for the given log level is non-existent or has a size that
    // is not within 100 chars of limit or over the limit...
    if (!file_map[level]->stream.is_open() ||
        file_map[level]->get_size() > max_filesize - 100)
    {
      string newlogpath = new_log_filepath(level);
      file_map[level]->init(newlogpath);
      file_map[level]->stream << "Header Format: HH-MM-SS-LLL "
          "<thread name>:<thread_id>:<thread_priority>\n";
      file_map[level]->stream.flush();
    }
  }

  /**
   *
   */
  string new_log_filepath(const LogLevel level) const
  {
    TimeStamp ts = elapsed_time();
    std::ostringstream name_builder;
    name_builder << log_dir << "/" << level_as_string(level, false) << "_"
        << ts.HH << "-" << ts.MM << "-" << ts.SS << ".log";
    return name_builder.str();
  }

  /**
   *
   */
  TimeStamp elapsed_time() const
  {
    long millis = startup_time.elapsed() / 1000;
    long hours = millis / (1000*60*60);
    long minutes = millis / (1000*60);
    long seconds = millis / 1000;
    millis = millis % 1000;

    TimeStamp ts;

    ts.HH = std::to_string(hours);
    if (hours > 9) ts.HH.insert(0, "0");

    ts.MM = std::to_string(minutes);
    if (minutes > 9) ts.MM.insert(0, "0");

    ts.SS = std::to_string(seconds);
    if (seconds > 9) ts.SS.insert(0, "0");

    ts.LLL = std::to_string(millis);
    if (millis > 99) ts.LLL.insert(0, "00");
    else if (millis > 9) ts.LLL.insert(0, "0");

    return ts;
  }

  /**
   *
   */
  string level_as_string(const LogLevel level, bool uppercase) const
  {
    switch (level)
    {
      case NOTICE:
        return uppercase ? "NOTICE" : "notice";
      case WARNING:
        return uppercase ? "WARNING" : "warning";
      case ERROR:
        return uppercase ? "ERROR" : "error";
      case CRITICAL:
        return uppercase ? "CRITICAL" : "critical";
    }
  }
};

#endif
