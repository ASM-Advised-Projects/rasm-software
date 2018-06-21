/**
 * http_server.h
 *
 * Defines the RASM's HTTP server subsystem.
 * This file implements the HTTP server backend.
 */

#ifndef HTTP_SERVER_INCLUDED
#define HTTP_SERVER_INCLUDED

#include "configuration.hpp"

#include <string>
#include <map>
#include <Poco/Path.h>
#include <Poco/File.h>
#include <Poco/Thread.h>
#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPServerParams.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPServerResponse.h>

namespace web
{

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
 * Handles HTTP GET requests by sending the file that corresponds to the
 * request URI. If the file doesn't exist then a 404 Not Found response is
 * sent.
 */
class GETRequestHandler : public pnet::HTTPRequestHandler
{
private:
  std::map<string, string> ext_to_types;

public:
  GETRequestHandler()
  {
    ext_to_types[".html"] = "text/html";
    ext_to_types[".css"] = "text/css";
    ext_to_types[".js"] = "text/javascript";
    ext_to_types[".png"] = "text/png";
    ext_to_types[".jpg"] = "text/jpeg";
    ext_to_types[".jpeg"] = "text/jpeg";
  }

  void handleRequest(pnet::HTTPServerRequest &request, pnet::HTTPServerResponse &response)
  {
    Poco::Path filepath = Poco::Path(DOC_ROOT + request.getURI());
    filepath.makeAbsolute();
    Poco::File requested_file = Poco::File(filepath);
    if (requested_file.isFile() && !requested_file.isHidden() && requested_file.exists())
    {
      response.setStatus(pnet::HTTPResponse::HTTPStatus::HTTP_OK);

      string fileext = filepath.getExtension();
      if (ext_to_types.count(fileext) > 0)
        response.setContentType(ext_to_types[fileext]);
      else
        response.setContentType("text/plain");

      response.setContentLength(requested_file.getSize());
      response.sendFile(requested_file.path(), response.getContentType());
    }
    else
    {
      response.setStatus(pnet::HTTPResponse::HTTPStatus::HTTP_NOT_FOUND);
      response.setContentType("text/plain");
      response.setContentLength(0);
      response.send();
    }
  }
};

/**
 * Handles HTTP POST requests by parsing their headers to get the type of form
 * and subsequently parsing their data to update the backend with. Things like
 * the RASM's configuration or user-review databases are updated.
 */
class POSTRequestHandler : public pnet::HTTPRequestHandler
{
private:

public:
  POSTRequestHandler()
  {

  }

  void handleRequest(pnet::HTTPServerRequest &request, pnet::HTTPServerResponse &response)
  {
    //request.
  }
};

/**
 * Handles all HTTP requests by sending a 403 Forbidden response.
 */
class OtherRequestHandler : public pnet::HTTPRequestHandler
{
public:
  void handleRequest(pnet::HTTPServerRequest &request, pnet::HTTPServerResponse &response)
  {
    response.setStatus(pnet::HTTPResponse::HTTPStatus::HTTP_FORBIDDEN);
    response.setContentType("text/plain");
    response.setContentLength(0);
    response.send();
  }
};

class HandlerFactory : public pnet::HTTPRequestHandlerFactory
{
private:
  GETRequestHandler *get_handler;
  POSTRequestHandler *post_handler;
  OtherRequestHandler *other_handler;

public:
  HandlerFactory()
  {
    get_handler = new GETRequestHandler();
    post_handler = new POSTRequestHandler();
    other_handler = new OtherRequestHandler();
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

  ~HandlerFactory()
  {
    get_handler->~GETRequestHandler();
    post_handler->~POSTRequestHandler();
    other_handler->~OtherRequestHandler();
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

  virtual pnet::HTTPServerParams* load_params()
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

}  // namespace web

#endif
