/**
 * Unit tests for the rasm2/util/time.hpp file.
 */

#ifndef TEST_RASM2_UTIL_TIME_HPP
#define TEST_RASM2_UTIL_TIME_HPP

#include <unistd.h>
#include <cppunit/extensions/HelperMacros.h>
#include "rasm2/util/time.hpp"

class TimeFunctionTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(TimeFunctionTests);
  CPPUNIT_TEST(testSystemMicros);
  CPPUNIT_TEST(testSystemMillis);
  CPPUNIT_TEST(testSystemSeconds);
  CPPUNIT_TEST(testProgramMicros);
  CPPUNIT_TEST(testProgramMillis);
  CPPUNIT_TEST(testProgramSeconds);
  CPPUNIT_TEST_SUITE_END();

public:
  void testSystemMicros()
  {
    unsigned int t1 = SystemTime::current_micros();
    usleep(50*1000);
    unsigned int t2 = SystemTime::current_micros();
    CPPUNIT_ASSERT(t2 - t1 >= 50000);
    CPPUNIT_ASSERT(t2 - t1 <= 60000);
  }

  void testSystemMillis()
  {
    unsigned int t1 = SystemTime::current_millis();
    usleep(50*1000);
    unsigned int t2 = SystemTime::current_millis();
    CPPUNIT_ASSERT(t2 - t1 >= 50);
    CPPUNIT_ASSERT(t2 - t1 <= 60);
  }

  void testSystemSeconds()
  {
    unsigned int t1 = SystemTime::current_seconds();
    usleep(1000*1000);
    unsigned int t2 = SystemTime::current_seconds();
    CPPUNIT_ASSERT(t2 - t1 == 1);
  }

  void testProgramMicros()
  {
    unsigned int t1 = ProgramTime::current_micros();
    usleep(50*1000);
    unsigned int t2 = ProgramTime::current_micros();
    CPPUNIT_ASSERT(t2 - t1 >= 50000);
    CPPUNIT_ASSERT(t2 - t1 <= 60000);
  }

  void testProgramMillis()
  {
    unsigned int t1 = ProgramTime::current_millis();
    usleep(50*1000);
    unsigned int t2 = ProgramTime::current_millis();
    CPPUNIT_ASSERT(t2 - t1 >= 50);
    CPPUNIT_ASSERT(t2 - t1 <= 60);
  }

  void testProgramSeconds()
  {
    unsigned int t1 = ProgramTime::current_seconds();
    usleep(1000*1000);
    unsigned int t2 = ProgramTime::current_seconds();
    CPPUNIT_ASSERT(t2 - t1 == 1);
  }
};

class StopwatchTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(StopwatchTests);
  CPPUNIT_TEST(testElapsedOnConstruction);
  CPPUNIT_TEST(testOneLevelTiming);
  CPPUNIT_TEST(testTwoLevelTiming);
  CPPUNIT_TEST(testFiveLevelTiming);
  CPPUNIT_TEST(testExtraStart);
  CPPUNIT_TEST(testExtraStop);
  CPPUNIT_TEST(testExtraStartAfterUsage);
  CPPUNIT_TEST(testExtraStopAfterUsage);
  CPPUNIT_TEST_SUITE_END();

  Stopwatch *sw1;

public:
  void setUp()
  {
    sw1 = new Stopwatch();
  }

  void tearDown()
  {
    delete sw1;
  }

  void testElapsedOnConstruction()
  {
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
  }

  void testOneLevelTiming()
  {
    sw1->start();
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->stop();
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);

    sw1->start();
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
    sw1->stop();
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
  }

  void testTwoLevelTiming()
  {
    sw1->start();
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->start();
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->stop();
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
    sw1->stop();
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 200);
    CPPUNIT_ASSERT(sw1->elapsed() <= 220);
  }

  void testFiveLevelTiming()
  {
    sw1->start();  // 1
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->start();  // 2
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->start();  // 3
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->start();  // 4
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() == 0);
    sw1->start();  // 5
    CPPUNIT_ASSERT(sw1->elapsed() == 0);

    sw1->stop();  // 5
    CPPUNIT_ASSERT(sw1->elapsed() <= 10);
    sw1->stop();  // 4
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 100);
    CPPUNIT_ASSERT(sw1->elapsed() <= 110);
    sw1->stop();  // 3
    CPPUNIT_ASSERT(sw1->elapsed() >= 300);
    CPPUNIT_ASSERT(sw1->elapsed() <= 330);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 300);
    CPPUNIT_ASSERT(sw1->elapsed() <= 330);
    sw1->stop();  // 2
    CPPUNIT_ASSERT(sw1->elapsed() >= 500);
    CPPUNIT_ASSERT(sw1->elapsed() <= 550);
    usleep(100*1000);
    CPPUNIT_ASSERT(sw1->elapsed() >= 500);
    CPPUNIT_ASSERT(sw1->elapsed() <= 550);
    sw1->stop();  // 1
    CPPUNIT_ASSERT(sw1->elapsed() >= 700);
    CPPUNIT_ASSERT(sw1->elapsed() <= 770);
  }

  void testExtraStart()
  {
    for (int i = 0; i < 5; i++)
      sw1->start();
    CPPUNIT_ASSERT_THROW(sw1->start(), std::runtime_error);
  }

  void testExtraStop()
  {
    CPPUNIT_ASSERT_THROW(sw1->stop(), std::runtime_error);
  }

  void testExtraStartAfterUsage()
  {
    sw1->start();
    sw1->stop();
    sw1->start();
    sw1->start();
    sw1->stop();
    sw1->start();
    sw1->stop();
    sw1->stop();

    for (int i = 0; i < 5; i++)
      sw1->start();
    CPPUNIT_ASSERT_THROW(sw1->start(), std::runtime_error);
  }

  void testExtraStopAfterUsage()
  {
    sw1->start();
    sw1->stop();
    sw1->start();
    sw1->start();
    sw1->stop();
    sw1->start();
    sw1->stop();
    sw1->stop();

    CPPUNIT_ASSERT_THROW(sw1->stop(), std::runtime_error);
  }
};

#endif
