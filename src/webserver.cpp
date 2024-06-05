#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "webserver.h"


AsyncWebServer server(80);

void webserver_begin()
{
    server.on( "/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send( 200, "text/plain", "Hello, world!" );
    });
}


/* Modeline for ViM {{{
 * vim:set ts=4:
 * vim600:fdm=marker fdl=0 fdc=3:
 * }}} */
