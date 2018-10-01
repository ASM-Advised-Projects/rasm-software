/**
 * Defines all interactive and non-interactive tests for the http subsystem.
 */

#ifndef HTTP_TESTS_H
#define HTTP_TESTS_H

#include "http/http_server.hpp"
#include "lest.hpp"
#include <unistd.h>

namespace http_tests
{

const lest::test nonactive[] =
{

    CASE( "HTTP server constructs and destructs without error" )
    {
      EXPECT_NO_THROW(RasmHttpServer server);
      RasmHttpServer server;
      EXPECT_NO_THROW(server.~RasmHttpServer());
    },

};

const lest::test active[] =
{

  CASE( "HTTP server runs and properly responds to requests" )
  {
    RasmHttpServer server;
    std::cout << "press any key to stop the 'test'" << std::endl;
    std::string temp;
    std::cin >> temp;
    server.~RasmHttpServer();
  },

};

}  // end namespace

#endif
