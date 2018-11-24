/**
 * Unit tests for the rasm2/battery.hpp file.
 */

#ifndef TEST_RASM2_BATTERY_HPP
#define TEST_RASM2_BATTERY_HPP

#include <iostream>
#include <unistd.h>
#include <vector>
#include <cppunit/extensions/HelperMacros.h>
#include <Poco/Semaphore.h>
#include "rasm2/battery.hpp"

/**
 * A subclass of BatteryEstimator that exposes all it's protected methods as
 * public methods so they can be tested.
 */
class BatteryEstimatorSub : protected BatteryEstimator, public Poco::Util::TimerTask
{
private:
  Poco::Semaphore binary_sema;
  int voltage_reads;

public:
  BatteryEstimatorSub(bool use_charge_data)
  : BatteryEstimator(std::bind(use_charge_data ?
      &BatteryEstimatorSub::charge_voltage_getter :
      &BatteryEstimatorSub::discharge_voltage_getter, this))
  , binary_sema(0, 1)
  , voltage_reads(0)
  {
  }

  // required by TimerTask
  virtual void run() { }

  // returns the number of calls to the voltage getter that have occured so far
  int read_count()
  {
    return voltage_reads;
  }

  // returns only when the voltage getter has been called a total of n times
  void block_till_nth_read(int n)
  {
    while (voltage_reads < n)
      binary_sema.wait();
  }

  // returns very similar voltage to what's contained in the charge_data.csv file
  double charge_voltage_getter()
  {
    ++voltage_reads;
    binary_sema.set();
    return 0;
  }

  // returns very similar voltage to what's contained in the charge_data.csv file
  double discharge_voltage_getter()
  {
    ++voltage_reads;
    binary_sema.set();
    return 0;
  }

  static int index_of_min(const vector<double> &list)
  {
    return BatteryEstimator::index_of_min(list);
  }

  static void parse_number_list(const string &num_list, vector<double> &vals)
  {
    BatteryEstimator::parse_number_list(num_list, vals);
  }

  static void load_battery_data(const string &filepath, vector<BatteryDatum> &rows)
  {
    BatteryEstimator::load_battery_data(filepath, rows);
  }

  static void load_battery_model(const string &filepath,
      vector<BatteryDatum> &particles, int num_particles)
  {
    BatteryEstimator::load_battery_model(filepath, particles, num_particles);
  }

  virtual void read_voltage_external()
  {
    BatteryEstimator::read_voltage(*this);
  }

  virtual void estimate_state_external()
  {
    BatteryEstimator::estimate_state(*this);
  }
};


class BatteryEstimatorTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(BatteryEstimatorTests);
  //CPPUNIT_TEST(testConstruction);
  //CPPUNIT_TEST(testReadVoltage);
  //CPPUNIT_TEST(testAutoReadVoltage);
  //CPPUNIT_TEST(testAutoEstimateState);
  CPPUNIT_TEST(testLoadBatteryData);
  CPPUNIT_TEST(testLoadBatteryModel);
  CPPUNIT_TEST(testParseNumberList);
  CPPUNIT_TEST(testIndexOfMin);
  CPPUNIT_TEST_SUITE_END();

  double fake_voltage_getter()
  {
    return 0;
  }

public:


  void setUp()
  {
  }

  void tearDown()
  {
  }

  /*void testReadVoltage()
  {
    BatteryEstimatorSub estimator(true);
    estimator.read_voltage_external();
    CPPUNIT_ASSERT(estimator.read_count() == 1);
  }

  void testAutoReadVoltage()
  {
    BatteryEstimatorSub estimator(true);
    usleep(350*1000);  // time for three reads
    CPPUNIT_ASSERT(estimator.read_count() == 3);

    usleep(200*1000);  // two more reads
    CPPUNIT_ASSERT(estimator.read_count() == 5);
  }

  void testAutoEstimateState()
  {
    BatteryEstimatorSub estimator(true);
    usleep(550*1000);  // time for one estimation
    CPPUNIT_ASSERT(estimator.read_count() == 1);

    usleep(500*1000);  // one more estimation
    CPPUNIT_ASSERT(estimator.read_count() == 2);
  }*/

  void testLoadBatteryData()
  {
    std::vector<BatteryEstimator::BatteryDatum> charge_data_rows;
    BatteryEstimatorSub::load_battery_data(
        "/Users/joepollard1/Projects/RASM/Software/test/filesysroot/rasm_2_1/"
        "data/battery/charge_data.csv", charge_data_rows);

    std::array<double, 11> volts =
        {{ 10.5, 11.9, 11.2, 11.5, 11.8, 12.0, 12.3, 12.6, 13.0, 13.4, 14.0 }};
    for (int i = 0; i <= 10; ++i)
    {
      CPPUNIT_ASSERT(charge_data_rows[i].seconds == i*10);
      CPPUNIT_ASSERT(charge_data_rows[i].voltspersec == 0);
      CPPUNIT_ASSERT(charge_data_rows[i].volts == volts[i]);
    }

    std::vector<BatteryEstimator::BatteryDatum> discharge_data_rows;
    BatteryEstimatorSub::load_battery_data(
        "/Users/joepollard1/Projects/RASM/Software/test/filesysroot/rasm_2_1/"
        "data/battery/discharge_data.csv", discharge_data_rows);

    for (int i = 0; i <= 10; ++i)
    {
      CPPUNIT_ASSERT(discharge_data_rows[i].seconds == i*10);
      CPPUNIT_ASSERT(discharge_data_rows[i].volts == volts[10-i]);
      CPPUNIT_ASSERT(discharge_data_rows[i].voltspersec == 0);
    }
  }

  void testLoadBatteryModel()
  {
  }

  void testParseNumberList()
  {
    std::string str;
    std::vector<double> list;

    // empty string
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 0);

    // one element
    str = "5";
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 1);
    CPPUNIT_ASSERT(list[0] == 5);

    // many positive elements
    str = "7,53,1,0";
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 4);
    CPPUNIT_ASSERT(list[0] == 7);
    CPPUNIT_ASSERT(list[1] == 53);
    CPPUNIT_ASSERT(list[2] == 1);
    CPPUNIT_ASSERT(list[3] == 0);

    // positive and negative elements
    str = "-3,7,-71,0";
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 4);
    CPPUNIT_ASSERT(list[0] == -3);
    CPPUNIT_ASSERT(list[1] == 7);
    CPPUNIT_ASSERT(list[2] == -71);
    CPPUNIT_ASSERT(list[3] == 0);

    // with spaces
    str = " -3 , 7, -71 ,0 ";
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 4);
    CPPUNIT_ASSERT(list[0] == -3);
    CPPUNIT_ASSERT(list[1] == 7);
    CPPUNIT_ASSERT(list[2] == -71);
    CPPUNIT_ASSERT(list[3] == 0);

    // with extra comma
    str = "-3,7,-71,0,";
    BatteryEstimatorSub::parse_number_list(str, list);
    CPPUNIT_ASSERT(list.size() == 4);
    CPPUNIT_ASSERT(list[0] == -3);
    CPPUNIT_ASSERT(list[1] == 7);
    CPPUNIT_ASSERT(list[2] == -71);
    CPPUNIT_ASSERT(list[3] == 0);

    // with non-numeric data
    str = " -3 , 7, -71 ,0a";
    CPPUNIT_ASSERT_THROW(BatteryEstimatorSub::parse_number_list(str, list), Poco::SyntaxException);
    str = " -3 , i7, -71 ,0";
    CPPUNIT_ASSERT_THROW(BatteryEstimatorSub::parse_number_list(str, list), Poco::SyntaxException);
  }

  void testIndexOfMin()
  {
    std::vector<double> list;

    // empty list
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == -1);

    // one element
    list.push_back(10);
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == 0);

    // positive and negative numbers w/min as first element
    list.clear();
    list.push_back(-5);
    list.push_back(0);
    list.push_back(-2);
    list.push_back(1);
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == 0);

    // duplicate min
    list.push_back(-5);
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == 0);

    // min as last element
    list.push_back(-6);
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == list.size()-1);

    // just positive numbers
    list.clear();
    list.push_back(3);
    list.push_back(2);
    list.push_back(3);
    list.push_back(3);
    CPPUNIT_ASSERT(BatteryEstimatorSub::index_of_min(list) == 1);
  }
};

#endif
