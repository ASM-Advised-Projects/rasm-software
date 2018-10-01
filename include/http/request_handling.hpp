/**
 * Defines the GETRequestHandler, POSTRequestHandler, and OtherRequestHandler.
 */

#ifndef REQUEST_HANDLING_INCLUDED
#define REQUEST_HANDLING_INCLUDED

#include <map>

#include <Poco/Path.h>
#include <Poco/File.h>
#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPServerParams.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPServerResponse.h>

#include "configuration.hpp"

namespace pnet = Poco::Net;
using std::string;


/**
 * Handles HTTP GET requests by sending the file that corresponds to the
 * request URI. If the file doesn't exist then a 404 Not Found response is
 * sent.
 */
class GETRequestHandler : public pnet::HTTPRequestHandler
{
private:
  std::map<string, string> ext_to_types;
  string root_dir;

public:
  GETRequestHandler(string root_dir) : root_dir(root_dir)
  {
    ext_to_types[".html"] = "text/html";
    ext_to_types[".css"] = "text/css";
    ext_to_types[".js"] = "text/javascript";
    ext_to_types[".png"] = "text/png";
    ext_to_types[".jpg"] = "text/jpg";
    ext_to_types[".jpeg"] = "text/jpeg";
  }

  void handleRequest(pnet::HTTPServerRequest &request, pnet::HTTPServerResponse &response)
  {
    // get the full path of the requested resource (file)
    Poco::Path filepath = Poco::Path(root_dir + request.getURI());
    filepath.makeAbsolute();
    Poco::File requested_file = Poco::File(filepath);

    // if the file exists...
    if (requested_file.isFile() && !requested_file.isHidden() && requested_file.exists())
    {
      // file doesn't exist - respond with 200 code and requested file
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
      // file doesn't exist - respond with 404 code
      response.setStatus(pnet::HTTPResponse::HTTPStatus::HTTP_NOT_FOUND);
      response.setContentType("text/plain");
      response.setContentLength(0);
      response.send();
    }
  }
};


/**
 * Handles HTTP POST requests by parsing its data as new settings and sending
 * them to the configuration subsystem.
 */
class POSTRequestHandler : public pnet::HTTPRequestHandler
{
private:
ConfigurationManager & config;

public:
  POSTRequestHandler()
  : config(ConfigurationManager::get_instance())
  {
  }

  void handleRequest(pnet::HTTPServerRequest &request, pnet::HTTPServerResponse &response)
  {
    //response.setStatus(pnet::HTTPResponse::HTTPStatus::HTTP_ACCEPTED);
    //response.setContentType("text/plain");
    //response.setContentLength(0);
    //response.send();
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

#endif
