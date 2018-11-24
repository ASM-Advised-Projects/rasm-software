/**
 * Unit tests for the rasm2/logging.hpp file.
 */

#ifndef TEST_RASM2_LOGGING_HPP
#define TEST_RASM2_LOGGING_HPP

#include <unistd.h>
#include <cppunit/extensions/HelperMacros.h>
#include "rasm2/logging.hpp"

/**
 * A subclass of LogManagerImpl that exposes all it's protected methods as
 * public methods so they can be tested.
 */
class LogManagerImplSub : protected LogManagerImpl
{
public:
  bool notice_flush;
  bool warning_flush;

  LogManagerImplSub() : LogManagerImpl() { }

  void flush_notice_log(Poco::Util::TimerTask &tt)
  {
    notice_flush = true;
    LogManagerImpl::flush_notice_log(tt);
  }

  void flush_warning_log(Poco::Util::TimerTask &tt)
  {
    warning_flush = true;
    LogManagerImpl::flush_warning_log(tt);
  }

  bool rotate_log_check(LogLevel level)
  {
    return LogManagerImpl::rotate_log_check(level);
  }

  string new_log_filepath(LogLevel level) const
  {
    return LogManagerImpl::new_log_filepath(level);
  }

  TimeStamp elapsed_time() const
  {
    return LogManagerImpl::elapsed_time();
  }

  string level_as_string(LogLevel level, bool uppercase) const
  {
    return LogManagerImpl::level_as_string(level, uppercase);
  }

  void log_message(LogManagerImpl::LogLevel level, const string &msg)
  {
    LogManagerImpl::log_message(level, msg);
  }

  std::map< LogManagerImpl::LogLevel, std::shared_ptr<LogFile> > get_file_map()
  {
    return file_map;
  }
};

class LogFileTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(LogFileTests);
  CPPUNIT_TEST(test);
  CPPUNIT_TEST_SUITE_END();

public:
  void setUp()
  {
  }

  void tearDown()
  {
  }

  void test()
  {
  }
};

class LogManagerTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(LogManagerTests);
  CPPUNIT_TEST(testFlushWarningLog);
  CPPUNIT_TEST(testFlushNoticeLog);
  CPPUNIT_TEST(testAutoFlush);
  CPPUNIT_TEST(testRotateLogCheck);
  CPPUNIT_TEST(testNewLogFilepath);
  CPPUNIT_TEST(testElapsedTime);
  CPPUNIT_TEST(testLevelAsString);
  CPPUNIT_TEST(testLogMessage1);
  CPPUNIT_TEST(testLogMessage2);
  CPPUNIT_TEST_SUITE_END();

  typedef LogManagerImpl::LogLevel Level;

public:
  void setUp()
  {
  }

  void tearDown()
  {
  }

  void testFlushWarningLog()
  {
  }

  void testFlushNoticeLog()
  {
  }

  void testAutoFlush()
  {
    LogManagerImplSub manager;
    manager.notice_flush = false;
    manager.warning_flush = false;

    usleep(1100*1000);
    CPPUNIT_ASSERT(manager.notice_flush);
    CPPUNIT_ASSERT(manager.warning_flush);
  }

  void testRotateLogCheck()
  {
  }

  void testNewLogFilepath()
  {
    LogManagerImplSub manager;

    std::array<int, 4> lengths = {{ 6, 7, 5, 8 }};
    std::array<Level, 4> levels = {{
      Level::NOTICE,
      Level::WARNING,
      Level::ERROR,
      Level::CRITICAL
    }};
    std::array<string, 4> names = {{ "notice", "warning", "error", "critical" }};

    for (int i = 0; i < 4; ++i)
    {
      TimeStamp t;
      do { t = manager.elapsed_time(); }
      while (std::atoi(t.LLL.c_str()) > 950);

      std::string filepath = manager.new_log_filepath(levels[i]);
      int name_length = lengths[i] + 13;
      string filename(filepath, filepath.size()-name_length, name_length);
      CPPUNIT_ASSERT(names[i] + "_" + t.HH + "-" + t.MM + "-" + t.SS + ".log" == filename);
    }
  }

  void testElapsedTime()
  {
    LogManagerImplSub manager;

    for (int trial = 0; trial < 5; ++trial)
    {
      TimeStamp time1 = manager.elapsed_time();
      CPPUNIT_ASSERT(time1.HH.length() == 2);
      CPPUNIT_ASSERT(time1.MM.length() == 2);
      CPPUNIT_ASSERT(time1.SS.length() == 2);
      CPPUNIT_ASSERT(time1.LLL.length() == 3);

      time1 = manager.elapsed_time();
      usleep(50*1000);
      TimeStamp time2 = manager.elapsed_time();

      int time1mil = std::atoi(time1.LLL.c_str());
      int time2mil = std::atoi(time2.LLL.c_str());
      if (time1mil < 940)
      {
        CPPUNIT_ASSERT(time2mil >= time1mil+50);
        CPPUNIT_ASSERT(time2mil <= time1mil+60);
        CPPUNIT_ASSERT(time2.SS == time1.SS);
      }
      else
      {
        int time1sec = std::atoi(time1.SS.c_str());
        if (time1sec < 59)
        {
          CPPUNIT_ASSERT(time2.MM == time1.MM);
        }
      }
      CPPUNIT_ASSERT(time2.HH == time1.HH);
    }
  }

  void testLevelAsString()
  {
    LogManagerImplSub manager;

    CPPUNIT_ASSERT("notice" == manager.level_as_string(Level::NOTICE, false));
    CPPUNIT_ASSERT("warning" == manager.level_as_string(Level::WARNING, false));
    CPPUNIT_ASSERT("error" == manager.level_as_string(Level::ERROR, false));
    CPPUNIT_ASSERT("critical" == manager.level_as_string(Level::CRITICAL, false));
    CPPUNIT_ASSERT("NOTICE" == manager.level_as_string(Level::NOTICE, true));
    CPPUNIT_ASSERT("WARNING" == manager.level_as_string(Level::WARNING, true));
    CPPUNIT_ASSERT("ERROR" == manager.level_as_string(Level::ERROR, true));
    CPPUNIT_ASSERT("CRITICAL" == manager.level_as_string(Level::CRITICAL, true));
  }

  void testLogMessage1()
  {
    LogManagerImplSub manager;

    string header = "00h-00m-00s-000l  name=main; id=0; priority=0";

    for (int i = 0; i < 5; ++i)
      manager.log_message(Level::CRITICAL, "Test log.");
    manager.get_file_map()[Level::WARNING]->stream.flush();
    manager.get_file_map()[Level::NOTICE]->stream.flush();
    CPPUNIT_ASSERT(manager.get_file_map()[Level::CRITICAL]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::ERROR]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::WARNING]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::NOTICE]->get_size() > 5*(header.size()+9));

    for (int i = 0; i < 5; ++i)
      manager.log_message(Level::ERROR, "Test log.");
    manager.get_file_map()[Level::WARNING]->stream.flush();
    manager.get_file_map()[Level::NOTICE]->stream.flush();
    CPPUNIT_ASSERT(manager.get_file_map()[Level::CRITICAL]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::ERROR]->get_size() > 10*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::WARNING]->get_size() > 10*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::NOTICE]->get_size() > 10*(header.size()+9));

    for (int i = 0; i < 5; ++i)
      manager.log_message(Level::WARNING, "Test log.");
    manager.get_file_map()[Level::WARNING]->stream.flush();
    manager.get_file_map()[Level::NOTICE]->stream.flush();
    CPPUNIT_ASSERT(manager.get_file_map()[Level::CRITICAL]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::ERROR]->get_size() > 10*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::WARNING]->get_size() > 15*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::NOTICE]->get_size() > 15*(header.size()+9));

    for (int i = 0; i < 5; ++i)
      manager.log_message(Level::NOTICE, "Test log.");
    manager.get_file_map()[Level::NOTICE]->stream.flush();
    CPPUNIT_ASSERT(manager.get_file_map()[Level::CRITICAL]->get_size() > 5*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::ERROR]->get_size() > 10*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::WARNING]->get_size() > 15*(header.size()+9));
    CPPUNIT_ASSERT(manager.get_file_map()[Level::NOTICE]->get_size() > 20*(header.size()+9));
  }

  void testLogMessage2()
  {
    LogManagerImplSub manager;

    usleep(1000*1000);  // ensure that the log filename will change

    string header = "00h-00m-00s-000l  name=main; id=0; priority=0";
    string msg = "empty";
    int num_logs = 10000 / (header.size() + msg.size());
    for (int i = 0; i < num_logs; ++i)
      manager.log_message(Level::NOTICE, msg);
  }
};

#endif
