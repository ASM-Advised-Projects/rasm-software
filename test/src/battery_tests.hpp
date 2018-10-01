/**
 * Defines all interactive and non-interactive tests for the battery subsytem.
 */

#ifndef BATTERY_TESTS_H
#define BATTERY_TESTS_H

#include "battery.hpp"
#include "lest.hpp"
#include <unistd.h>

namespace battery_tests
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
