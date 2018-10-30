/**
 * Runs all unit tests.
 */

// util
#include "test/rasm2/util/time.hpp"
#include "test/rasm2/util/circular_array.hpp"
#include "test/rasm2/util/signal_processing.hpp"
CPPUNIT_TEST_SUITE_REGISTRATION(StopwatchTests);
CPPUNIT_TEST_SUITE_REGISTRATION(TimeFunctionTests);
CPPUNIT_TEST_SUITE_REGISTRATION(CircularArrayTests);
CPPUNIT_TEST_SUITE_REGISTRATION(CausalLTIFilterTests);
CPPUNIT_TEST_SUITE_REGISTRATION(DifferentiatorTests);
CPPUNIT_TEST_SUITE_REGISTRATION(IntegratorTests);

// batt, config, log
////#include "test/rasm2/battery.hpp"
#include "test/rasm2/configuration.hpp"
#include "test/rasm2/logging.hpp"
//CPPUNIT_TEST_SUITE_REGISTRATION(BatteryEstimatorTests);
//CPPUNIT_TEST_SUITE_REGISTRATION(ConfigGroupTests);
CPPUNIT_TEST_SUITE_REGISTRATION(ConfigurationManagerTests);
//CPPUNIT_TEST_SUITE_REGISTRATION(LogFileTests);
CPPUNIT_TEST_SUITE_REGISTRATION(LogManagerTests);

// peripheral
//#include "test/rasm2/.hpp"
//CPPUNIT_TEST_SUITE_REGISTRATION();

// http
//#include "test/rasm2/.hpp"
//CPPUNIT_TEST_SUITE_REGISTRATION();

// vision
//#include "test/rasm2/.hpp"
//CPPUNIT_TEST_SUITE_REGISTRATION();

#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>

int main(int argc, char **argv)
{
  CppUnit::TextUi::TestRunner runner;
  CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
  runner.addTest(registry.makeTest());
  runner.run();
  return 0;
}
