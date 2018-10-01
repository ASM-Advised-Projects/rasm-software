/**
 * Defines all interactive and non-interactive tests for the configuration subsytem.
 */

#ifndef CONFIG_TESTS_H
#define CONFIG_TESTS_H

#include "configuration.hpp"
#include "lest.hpp"
#include <unistd.h>

namespace config_tests
{

const lest::test nonactive[] =
{

    CASE( "" )
    {

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
