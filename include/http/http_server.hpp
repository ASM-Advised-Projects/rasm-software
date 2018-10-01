/**
 * Defines the HandlerFactory and RasmHttpServer classes.
 */

#ifndef HTTP_SERVER_INCLUDED
#define HTTP_SERVER_INCLUDED

#include "request_handling.hpp"
#include "configuration.hpp"

namespace pnet = Poco::Net;
using std::string;

#define HTTP_SERVER_PORT 8080
#define DOMAIN_NAME string("rasm.app")
#define DOC_ROOT string("/root/rasm/httpserver/doc_root")
#define MAX_CLIENTS_QUEUED 5
#define MAX_THREADS 5
#define LOG_LEVEL 0

#define THREAD_PRIORITY Poco::Thread::Priority::PRIO_LOW


/**
 * Overrides the HTTPRequestHandlerFactory::createRequestHandler method in
 * order to return the appropriate request handler. Each handler is only created
 * once.
 */
class HandlerFactory : public pnet::HTTPRequestHandlerFactory
{
private:

  GETRequestHandler *get_handler;
  POSTRequestHandler *post_handler;
  OtherRequestHandler *other_handler;

public:
  HandlerFactory()
  {
    get_handler = new GETRequestHandler(ConfigurationManager::get_instance().rwd);
    post_handler = new POSTRequestHandler();
    other_handler = new OtherRequestHandler();
  }

  ~HandlerFactory()
  {
    get_handler->~GETRequestHandler();
    post_handler->~POSTRequestHandler();
    other_handler->~OtherRequestHandler();
  }

  virtual pnet::HTTPRequestHandler* createRequestHandler(const pnet::HTTPServerRequest &request)
  {
    if (request.getMethod() == "GET")
      return get_handler;
    else if (request.getMethod() == "POST")
      return post_handler;
    else
      return other_handler;
  }
};


/**
 * This server hosts local documents and dynamically created data using either
 * persistent or non-persistent connections. It also processes form submits
 * (POST requests) in order to configure the RASM.
 */
class RasmHttpServer
{
private:
  pnet::HTTPServer *httpserver;
  pnet::ServerSocket socket;

  pnet::HTTPServerParams* load_params()
  {
    pnet::HTTPServerParams *serverparams = new pnet::HTTPServerParams;

    serverparams->setServerName(DOMAIN_NAME + ":" + std::to_string(HTTP_SERVER_PORT));
    serverparams->setMaxQueued(MAX_CLIENTS_QUEUED);
    serverparams->setMaxThreads(MAX_THREADS);
    serverparams->setThreadPriority(THREAD_PRIORITY);

    return serverparams;
  }

public:
  RasmHttpServer()
  {
    Poco::Net::SocketAddress localAddress(
        Poco::Net::AddressFamily::IPv4,
        "127.0.0.1",
        HTTP_SERVER_PORT
    );
    socket.bind(localAddress);
    socket.listen();

    httpserver = new pnet::HTTPServer(new HandlerFactory(), socket, load_params());
  }

  ~RasmHttpServer()
  {
    httpserver->~HTTPServer();
    socket.close();
    socket.~ServerSocket();
  }
};

#endif
