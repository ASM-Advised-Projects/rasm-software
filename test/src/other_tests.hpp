/**
 * Defines all interactive and non-interactive tests for non-subsytem files.
 */

#ifndef OTHER_TESTS_H
#define OTHER_TESTS_H

#include "time.hpp"
#include "lest.hpp"
#include <unistd.h>

namespace other_tests
{

const lest::test nonactive[] =
{

    CASE( "RasmTime methods are accurate" )
    {
      RasmTime & rt = RasmTime::get_instance();
      int millis1 = rt.current_time_millis();
      int seconds1 = rt.current_time_seconds();
      usleep(20*1000);
      int millis2 = rt.current_time_millis();
      int seconds2 = rt.current_time_seconds();

      EXPECT( (1000*seconds1 >= millis1 && 1000*seconds1-10 <= millis1) );
      EXPECT( (1000*seconds2 >= millis2 && 1000*seconds2-10 <= millis2) );
      EXPECT( (millis2-millis1 >= 20 && millis2-millis1 <= 30) );
    },

    CASE( "Stopwatch constructs and destructs without exception" )
    {
      EXPECT_NO_THROW(Stopwatch sw);
      Stopwatch sw;
      EXPECT_NO_THROW(sw.~Stopwatch());
    },

    CASE( "Stopwatch throws error on start within 5th level" )
    {
      Stopwatch sw;
      sw.start();
      sw.start();
      sw.start();
      sw.start();
      sw.start();
      EXPECT_THROWS(sw.start());
    },

    CASE( "Stopwatch throws error on stop within 0th level" )
    {
      Stopwatch sw;
      EXPECT_THROWS(sw.stop());
    },

    CASE( "Stopwatch 0th and 1st level elapsed times are accurate" )
    {
      Stopwatch sw;

      EXPECT(sw.elapsed() == 0);  // 0th level; 1st not yet started

      sw.start();
      usleep(20*1000);
      EXPECT(sw.elapsed() == 0);  // 1st level; started but not stopped

      sw.stop();
      EXPECT((sw.elapsed() >= 20 && sw.elapsed() <= 30));  // 1st level; stopped
    },

    CASE( "Stopwatch all level elapsed times are accurate" )
    {
      Stopwatch sw;
      EXPECT(sw.elapsed() == 0);
      sw.start();
      sw.start();
      sw.start();
      sw.start();
      sw.start();
      usleep(20*1000);
      EXPECT(sw.elapsed() == 0);
      sw.stop();
      EXPECT((sw.elapsed() >= 20 && sw.elapsed() <= 30));  // level 5
      sw.stop();
      EXPECT((sw.elapsed() >= 20 && sw.elapsed() <= 30));  // level 4
      sw.start();
      usleep(20*1000);
      sw.stop();
      EXPECT((sw.elapsed() >= 40 && sw.elapsed() <= 50));  // level 4
      sw.stop();
      EXPECT((sw.elapsed() >= 40 && sw.elapsed() <= 50));  // level 3
      usleep(20*1000);
      sw.stop();
      usleep(20*1000);
      EXPECT((sw.elapsed() >= 60 && sw.elapsed() <= 70));  // level 2
      sw.stop();
      EXPECT((sw.elapsed() >= 80 && sw.elapsed() <= 90));  // level 1
    },

};

const lest::test active[] =
{

  CASE( "" )
  {

  },

};

}  // end namespace

#endif
