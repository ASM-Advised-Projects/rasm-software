/**
 * Unit tests for the rasm2/configuration.hpp file.
 */

#ifndef TEST_RASM2_CONFIGURATION_HPP
#define TEST_RASM2_CONFIGURATION_HPP

#include <cppunit/extensions/HelperMacros.h>
#include "rasm2/configuration.hpp"

class ConfigGroupTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(ConfigGroupTests);
  CPPUNIT_TEST(testCopyConstructor);
  CPPUNIT_TEST(testEqualsAssignment);
  CPPUNIT_TEST(testEqualsComparison);
  CPPUNIT_TEST(testNotEqualsComparison);
  CPPUNIT_TEST(testGetValue);
  CPPUNIT_TEST(testChangeValue);
  CPPUNIT_TEST(testPushToFile);
  CPPUNIT_TEST_SUITE_END();

public:
  void setUp()
  {
  }

  void tearDown()
  {
  }

  void testCopyConstructor()
  {
  }

  void testEqualsAssignment()
  {
  }

  void testEqualsComparison()
  {
  }

  void testNotEqualsComparison()
  {
  }

  void testGetValue()
  {
  }

  void testChangeValue()
  {
  }

  void testPushToFile()
  {
  }
};

class ConfigurationManagerTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(ConfigurationManagerTests);
  CPPUNIT_TEST(testAutoSync);
  CPPUNIT_TEST(testGetConfigGroup);
  CPPUNIT_TEST(testGetConfigValue);
  CPPUNIT_TEST(testChangeConfigValue);
  CPPUNIT_TEST(testSaveConfigChanges);
  CPPUNIT_TEST_SUITE_END();

  ConfigurationManagerImpl *manager;
  typedef ConfigurationManagerImpl::Group CMG;

public:
  void setUp()
  {
    manager = new ConfigurationManagerImpl();
  }

  void tearDown()
  {
    //delete manager;
  }

  void testAutoSync()
  {
  }

  void testGetConfigGroup()
  {
    auto group = manager->get_config_group(CMG::OTHER);
    CPPUNIT_ASSERT(group->getString("other_test") == "test");

    group = manager->get_config_group(CMG::LOGGING);
    CPPUNIT_ASSERT(group->getString("logging_test") == "test");

    group = manager->get_config_group(CMG::BATTERY);
    CPPUNIT_ASSERT(group->getString("battery_test") == "test");

    group = manager->get_config_group(CMG::VISION);
    CPPUNIT_ASSERT(group->getString("vision_test") == "test");

    group = manager->get_config_group(CMG::CONTROL);
    CPPUNIT_ASSERT(group->getString("control_test") == "test");

    group = manager->get_config_group(CMG::PERIPHERY);
    CPPUNIT_ASSERT(group->getString("periphery_test") == "test");

    group = manager->get_config_group(CMG::HTTP);
    CPPUNIT_ASSERT(group->getString("http_test") == "test");

    group = manager->get_config_group(CMG::SHELL);
    CPPUNIT_ASSERT(group->getString("shell_test") == "test");
  }

  void testGetConfigValue()
  {
    bool exists;
    std::string value;

    value = "hi";
    exists = manager->get_config_value(CMG::OTHER, "a_nonexistent_key", value);
    CPPUNIT_ASSERT(!exists);
    CPPUNIT_ASSERT(value == "");

    exists = manager->get_config_value(CMG::OTHER, "other_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::LOGGING, "logging_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::BATTERY, "battery_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::VISION, "vision_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::CONTROL, "control_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::PERIPHERY, "periphery_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::HTTP, "http_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->get_config_value(CMG::SHELL, "shell_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");
  }

  void testChangeConfigValue()
  {
    bool exists;
    string value;

    exists = manager->change_config_value(CMG::OTHER, "a_nonexistent_key", "blaa");
    CPPUNIT_ASSERT(!exists);
    exists = manager->get_config_value(CMG::OTHER, "a_nonexistent_key", value);
    CPPUNIT_ASSERT(!exists);

    exists = manager->change_config_value(CMG::OTHER, "other_test", "test2");
    CPPUNIT_ASSERT(exists);
    exists = manager->get_config_value(CMG::OTHER, "other_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test2");

    exists = manager->change_config_value(CMG::OTHER, "other_test", "test");
    CPPUNIT_ASSERT(exists);
    exists = manager->get_config_value(CMG::OTHER, "other_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");

    exists = manager->change_config_value(CMG::VISION, "vision_test", "test2");
    CPPUNIT_ASSERT(exists);
    exists = manager->get_config_value(CMG::VISION, "vision_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test2");

    exists = manager->change_config_value(CMG::VISION, "vision_test", "test");
    CPPUNIT_ASSERT(exists);
    exists = manager->get_config_value(CMG::VISION, "vision_test", value);
    CPPUNIT_ASSERT(exists);
    CPPUNIT_ASSERT(value == "test");
  }

  void testSaveConfigChanges()
  {
  }
};

#endif
