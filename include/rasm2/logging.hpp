/**
 * Implements the RASM's logging subsystem.
 * Defines the LogFile, LogManagerImpl, and LogManager (singleton) classes.
 */

#ifndef RASM2_LOGGING_HPP
#define RASM2_LOGGING_HPP

#include <fstream>
#include <sstream>
#include <sys/statvfs.h>

#include <Poco/Thread.h>
#include <Poco/Mutex.h>
#include <Poco/File.h>
#include <Poco/NumberParser.h>
#include <Poco/Util/TimerTask.h>
#include <Poco/Util/TimerTaskAdapter.h>

#include "configuration.hpp"
#include "util/time.hpp"

using std::string;

/**
 * For internal use by the LogManager class.
 * Represents a text file used for logging.
 */
class LogFile
{
public:
  string path;
  std::ofstream stream;

  /**
   * Sets the path field to the given filepath and opens the corresponding file
   * in the filesystem for appending. If the file doesn't exist then it will be
   * created. If this LogFile instance already has an open file then it will
   * first be closed.
   */
  void init(string &filepath)
  {
    close();
    path = filepath;
    stream.open(path, std::ofstream::out | std::ofstream::app);
  }

  /**
   * Closes the currently opened file, if there is one. The path field will be
   * set to an empty string.
   */
  void close()
  {
    if (stream.is_open())
    {
      stream.flush();
      stream.close();
    }
    path = "";
  }

  /**
   * Returns the size, in bytes, of the currently opened file. If a file isn't
   * open, then 0 is returned.
   */
  unsigned long get_size() const
  {
    if (stream.is_open())
    {
      std::ifstream in(path, std::ofstream::ate | std::ofstream::binary);
      return in.tellg();
    }
    return 0;
  }
};


/**
 * Represents a relative time with string hour, minute, second, and millisecond
 * fields.
 */
struct TimeStamp
{
  string HH;
  string MM;
  string SS;
  string LLL;
};


/**
 * This class represents a singleton that initializes and manages a logging
 * system which can be used anywhere else in the program to submit log messages
 * to the log filesystem. It handles file & directory creation/naming, provides
 * log priority levels, automatically rotates log files, and buffers log
 * messages. Instances of this class are thread-safe.
 *
 * Every program will have it's own log directory. The name of the that
 * directory will be an integer that is one larger than the current largest
 * integer directory name in the logging root folder. If no numbered directories
 * exist yet then the integer will be 1.
 *
 * Within the program's newly created directory, .log files are created for each
 * log level. Any one log file is rotated if it exceeds its size limit. The log
 * files will have the following naming convention:
 *   level_HH-MM-SS.log
 * Where level is that log file's priority level.
 *
 * There are four priority levels. From lowest to highest priority they are:
 * notice, warning, error, and critical. These are defined in the
 * LogManager::LogLevel enum.
 *
 * Each log message will be placed in the file corresponding to its priority
 * level along with the files for all lower priority levels. For example, an
 * error log message will be appended to the current log files for error,
 * warning, and notice logs.
 *
 * Each log message will have the following structure:
 *   <log priority>
 *   HH-MM-SS-LLL  <thread name>:<thread id>:<thread priority>
 *   <message>\n\n
 * The log priority level will only be placed at the start if the log message
 * has a priority that is higher than the log file's priority; otherwise that
 * first line won't exist for that log message.
 *
 * All messages going into the warning and notice log files will be buffered
 * before they are actually appended to the files. This periodic flushing of
 * log messages happens at the end of every buffer time interval or if the
 * number of logs of either type reaches 10 - whichever comes first. The length
 * of this interval is set by the configuration system. All critical and error
 * logs will be immediately written to their log files.
 */
class LogManagerImpl
{
public:
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

protected:
  std::map< LogManagerImpl::LogLevel, std::shared_ptr<LogFile> > file_map;
  Poco::Util::Timer flush_timer;
  string log_dir;
  long max_filesize;
  Poco::Mutex stream_mutex;
  int unflushed_notice_msgs;
  int unflushed_warning_msgs;

  /**
   * Flushes the notice log message stream to the current notice log file.
   */
  virtual void flush_notice_log(Poco::Util::TimerTask &)
  {
    unflushed_notice_msgs = 0;
    stream_mutex.lock();
    file_map[NOTICE]->stream.flush();
    stream_mutex.unlock();
  }

  /**
   * Flushes the warning log message stream to the current warning log file.
   */
  virtual void flush_warning_log(Poco::Util::TimerTask &)
  {
    unflushed_warning_msgs = 0;
    stream_mutex.lock();
    file_map[WARNING]->stream.flush();
    stream_mutex.unlock();
  }

  /**
   * Rotates the log file for the given log level if it is nearing or
   * exceeding the max log file size. If a log file for the level doesn't
   * exist yet then a file will be created. If a rotation occurs then true will
   * be returned; false if not.
   */
  virtual bool rotate_log_check(LogLevel level)
  {
    // if the file for the given log level is non-existent or has a size that
    // is not within 100 chars of limit or over the limit...
    if (!file_map[level]->stream.is_open() ||
        file_map[level]->get_size() > max_filesize - 100)
    {
      stream_mutex.lock();
      file_map[level]->stream.flush();  // in case of buffered notice or warning logs
      string newlogpath = new_log_filepath(level);
      file_map[level]->init(newlogpath);
      file_map[level]->stream << "Header Format:\n<HH>h-<MM>m-<SS>s-<LLL>l  "
          "name=<thread name>; id=<thread_id>; priority=<thread_priority>\n\n";
      file_map[level]->stream.flush();
      stream_mutex.unlock();
      return true;
    }
    return false;
  }

  /**
   * Returns the full file path of a new log file for the given priority level.
   * The name of the file is of the format level_HH-MM-SS.log. The level's name
   * is all lowercase.
   */
  virtual string new_log_filepath(LogLevel level) const
  {
    TimeStamp ts = elapsed_time();
    std::ostringstream name_builder;
    name_builder << log_dir << "/" << level_as_string(level, false) << "_"
        << ts.HH << "-" << ts.MM << "-" << ts.SS << ".log";
    return name_builder.str();
  }

  /**
   * Returns the elapsed time from when this logging singleton was initialized
   * as a TimeStamp struct.
   */
  virtual TimeStamp elapsed_time() const
  {
    unsigned long millis = ProgramTime::current_millis();
    unsigned long hours = millis / (1000*60*60);
    unsigned long minutes = (millis % (1000*60*60)) / (1000*60);
    unsigned long seconds = (millis % (1000*60)) / 1000;
    millis = millis % 1000;

    TimeStamp ts;

    ts.HH = std::to_string(hours);
    if (hours < 10) ts.HH.insert(0, "0");

    ts.MM = std::to_string(minutes);
    if (minutes < 10) ts.MM.insert(0, "0");

    ts.SS = std::to_string(seconds);
    if (seconds < 10) ts.SS.insert(0, "0");

    ts.LLL = std::to_string(millis);
    if (millis < 10) ts.LLL.insert(0, "00");
    else if (millis < 100) ts.LLL.insert(0, "0");

    return ts;
  }

  /**
   * Returns the string representation of the given log priority level. The
   * string returned is just the actual name of the level. If uppercase is set
   * to true, then the entire name will be uppercase; otherwise the entire name
   * will be lowercase.
   */
  virtual string level_as_string(LogLevel level, bool uppercase) const
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

public:
  LogManagerImpl(const LogManagerImpl &) = delete;
  void operator=(const LogManagerImpl &) = delete;

  /**
   * Creates a new log manager instance. Retrieves some configurations, sets up
   * a log directory, creates and opens a log file for each priority level, and
   * starts the log buffer flush timers.
   */
  LogManagerImpl()
  : flush_timer(Poco::Thread::PRIO_LOWEST)
  , unflushed_notice_msgs(0)
  , unflushed_warning_msgs(0)
  {
    // get logging configuration group
    ConfigurationManager &configs = ConfigurationManager::get_instance();
    const MapConfiguration *log_conf =
        configs.get_config_group(ConfigurationManagerImpl::Group::LOGGING);

    // get max log file size
    max_filesize = 1000 * log_conf->getInt64("max_filesize_kilobytes");

    // get directory names
    int dir_int = log_conf->getInt("next_log_num");
    int max_dir_int = log_conf->getInt("max_log_num");
    string rootdir;
    configs.get_config_value(ConfigurationManagerImpl::Group::OTHER, "root_working_dir", rootdir);
    rootdir += "/log/";

    // create new log directory file object (don't create actual directory yet)
    log_dir = rootdir + std::to_string(dir_int);
    Poco::File new_log_dir(log_dir);

    // remove similarly named directory if it exists
    if (new_log_dir.exists())
      new_log_dir.remove(true);

    // create new log directory
    new_log_dir.createDirectory();

    // increment the next log directory's name in configurations
    configs.change_config_value(ConfigurationManagerImpl::Group::LOGGING,
        "next_log_num", std::to_string((dir_int+1) % max_dir_int));

    // map each log level to a LogFile instance
    file_map[NOTICE].reset(new LogFile());
    file_map[WARNING].reset(new LogFile());
    file_map[ERROR].reset(new LogFile());
    file_map[CRITICAL].reset(new LogFile());

    // initialize LogFile classes
    for (const auto &pair : file_map)
      rotate_log_check(pair.first);

    // start the timer tasks for flushing the NOTICE and WARNING log buffers
    Poco::AutoPtr<Poco::Util::TimerTask> flush_task_1 =
        new Poco::Util::TimerTaskAdapter<LogManagerImpl>(
        *this, &LogManagerImpl::flush_notice_log);
    Poco::AutoPtr<Poco::Util::TimerTask> flush_task_2 =
        new Poco::Util::TimerTaskAdapter<LogManagerImpl>(
        *this, &LogManagerImpl::flush_warning_log);

    Poco::AutoPtr<Poco::Util::TimerTask> task_ptr_1(flush_task_1);
    Poco::AutoPtr<Poco::Util::TimerTask> task_ptr_2(flush_task_2);

    int interval = 1000;// * log_conf->getInt("flush_interval_sec");
    flush_timer.scheduleAtFixedRate(task_ptr_1, interval, interval);
    flush_timer.scheduleAtFixedRate(task_ptr_2, interval, interval);
  }

  virtual ~LogManagerImpl() noexcept
  {
    flush_timer.cancel();
    file_map[NOTICE]->stream.flush();
    file_map[WARNING]->stream.flush();
  }

  /**
   * Logs a message at the given priority level. The message will be placed in
   * the log files with priority levels equal to and lower than the given level.
   * The message will have this structure:
   *   <priority level>
   *   HH-MM-SS-LLL  <thread name>:<thread id>:<thread priority>
   *   <message>\n\n
   * That first line will only exist for the files whose priority levels are
   * lower than the given level.
   */
  virtual void log_message(LogLevel level, const string &msg)
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
    msg_builder << ts.HH << "h-" << ts.MM << "m-" << ts.SS << "s-" << ts.LLL
        << "l  name=" << thread_name << "; id=" << thread_id << "; priority="
        << thread_prio << "\n";
    msg_builder << msg << "\n\n";

    // log the headered message via switch case fall through
    stream_mutex.lock();
    switch (level)
    {
      case CRITICAL:
        file_map[CRITICAL]->stream << msg_builder.str();
        file_map[CRITICAL]->stream.flush();

      case ERROR:
        if (level != ERROR)
            file_map[ERROR]->stream << level_as_string(level, true) << "\n";
        file_map[ERROR]->stream << msg_builder.str();
        file_map[ERROR]->stream.flush();

      case WARNING:
        if (level != WARNING)
            file_map[WARNING]->stream << level_as_string(level, true) << "\n";
        file_map[WARNING]->stream << msg_builder.str();
        if (++unflushed_warning_msgs >= 10)
          file_map[WARNING]->stream.flush();

      case NOTICE:
        if (level != NOTICE)
            file_map[NOTICE]->stream << level_as_string(level, true) << "\n";
        file_map[NOTICE]->stream << msg_builder.str();
        ++unflushed_notice_msgs;
        if (++unflushed_notice_msgs >= 10)
          file_map[NOTICE]->stream.flush();
    }
    stream_mutex.unlock();

    rotate_log_check(level);
  }
};


/**
 *
 */
class LogManager
{
private:
  LogManagerImpl impl;

  LogManager()
  {
  }

public:
  LogManager(const LogManager &) = delete;
  void operator=(const LogManager &) = delete;

  /**
   * Returns this singleton's instance.
   */
  static LogManager & get_instance()
  {
    static LogManager instance;
    return instance;
  }

  void log_message(LogManagerImpl::LogLevel level, const string &msg)
  {
    impl.log_message(level, msg);
  }
};

#endif
