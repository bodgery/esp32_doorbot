/*
Copyright (c) 2025,  Timm Murray
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, 
      this list of conditions and the following disclaimer in the documentation 
      and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wiegand.h>

#include "config.h"


const char* version = "6";
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFBTCCAu2gAwIBAgIQS6hSk/eaL6JzBkuoBI110DANBgkqhkiG9w0BAQsFADBP\n" \
"MQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFy\n" \
"Y2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMTAeFw0yNDAzMTMwMDAwMDBa\n" \
"Fw0yNzAzMTIyMzU5NTlaMDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBF\n" \
"bmNyeXB0MQwwCgYDVQQDEwNSMTAwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK\n" \
"AoIBAQDPV+XmxFQS7bRH/sknWHZGUCiMHT6I3wWd1bUYKb3dtVq/+vbOo76vACFL\n" \
"YlpaPAEvxVgD9on/jhFD68G14BQHlo9vH9fnuoE5CXVlt8KvGFs3Jijno/QHK20a\n" \
"/6tYvJWuQP/py1fEtVt/eA0YYbwX51TGu0mRzW4Y0YCF7qZlNrx06rxQTOr8IfM4\n" \
"FpOUurDTazgGzRYSespSdcitdrLCnF2YRVxvYXvGLe48E1KGAdlX5jgc3421H5KR\n" \
"mudKHMxFqHJV8LDmowfs/acbZp4/SItxhHFYyTr6717yW0QrPHTnj7JHwQdqzZq3\n" \
"DZb3EoEmUVQK7GH29/Xi8orIlQ2NAgMBAAGjgfgwgfUwDgYDVR0PAQH/BAQDAgGG\n" \
"MB0GA1UdJQQWMBQGCCsGAQUFBwMCBggrBgEFBQcDATASBgNVHRMBAf8ECDAGAQH/\n" \
"AgEAMB0GA1UdDgQWBBS7vMNHpeS8qcbDpHIMEI2iNeHI6DAfBgNVHSMEGDAWgBR5\n" \
"tFnme7bl5AFzgAiIyBpY9umbbjAyBggrBgEFBQcBAQQmMCQwIgYIKwYBBQUHMAKG\n" \
"Fmh0dHA6Ly94MS5pLmxlbmNyLm9yZy8wEwYDVR0gBAwwCjAIBgZngQwBAgEwJwYD\n" \
"VR0fBCAwHjAcoBqgGIYWaHR0cDovL3gxLmMubGVuY3Iub3JnLzANBgkqhkiG9w0B\n" \
"AQsFAAOCAgEAkrHnQTfreZ2B5s3iJeE6IOmQRJWjgVzPw139vaBw1bGWKCIL0vIo\n" \
"zwzn1OZDjCQiHcFCktEJr59L9MhwTyAWsVrdAfYf+B9haxQnsHKNY67u4s5Lzzfd\n" \
"u6PUzeetUK29v+PsPmI2cJkxp+iN3epi4hKu9ZzUPSwMqtCceb7qPVxEbpYxY1p9\n" \
"1n5PJKBLBX9eb9LU6l8zSxPWV7bK3lG4XaMJgnT9x3ies7msFtpKK5bDtotij/l0\n" \
"GaKeA97pb5uwD9KgWvaFXMIEt8jVTjLEvwRdvCn294GPDF08U8lAkIv7tghluaQh\n" \
"1QnlE4SEN4LOECj8dsIGJXpGUk3aU3KkJz9icKy+aUgA+2cP21uh6NcDIS3XyfaZ\n" \
"QjmDQ993ChII8SXWupQZVBiIpcWO4RqZk3lr7Bz5MUCwzDIA359e57SSq5CCkY0N\n" \
"4B6Vulk7LktfwrdGNVI5BsC9qqxSwSKgRJeZ9wygIaehbHFHFhcBaMDKpiZlBHyz\n" \
"rsnnlFXCb5s8HKn5LsUgGvB24L7sGNZP2CX7dhHov+YhD+jozLW2p9W4959Bz2Ei\n" \
"RmqDtmiXLnzqTpXbI+suyCsohKRg6Un0RC47+cpiVwHiXZAW+cn8eiNIjqbVgXLx\n" \
"KPpdzvvtTnOPlC7SQZSYmdunr3Bf9b77AiC/ZidstK36dRILKz7OA54=\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFBjCCAu6gAwIBAgIRAIp9PhPWLzDvI4a9KQdrNPgwDQYJKoZIhvcNAQELBQAw\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw\n" \
"WhcNMjcwMzEyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n" \
"RW5jcnlwdDEMMAoGA1UEAxMDUjExMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\n" \
"CgKCAQEAuoe8XBsAOcvKCs3UZxD5ATylTqVhyybKUvsVAbe5KPUoHu0nsyQYOWcJ\n" \
"DAjs4DqwO3cOvfPlOVRBDE6uQdaZdN5R2+97/1i9qLcT9t4x1fJyyXJqC4N0lZxG\n" \
"AGQUmfOx2SLZzaiSqhwmej/+71gFewiVgdtxD4774zEJuwm+UE1fj5F2PVqdnoPy\n" \
"6cRms+EGZkNIGIBloDcYmpuEMpexsr3E+BUAnSeI++JjF5ZsmydnS8TbKF5pwnnw\n" \
"SVzgJFDhxLyhBax7QG0AtMJBP6dYuC/FXJuluwme8f7rsIU5/agK70XEeOtlKsLP\n" \
"Xzze41xNG/cLJyuqC0J3U095ah2H2QIDAQABo4H4MIH1MA4GA1UdDwEB/wQEAwIB\n" \
"hjAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwEgYDVR0TAQH/BAgwBgEB\n" \
"/wIBADAdBgNVHQ4EFgQUxc9GpOr0w8B6bJXELbBeki8m47kwHwYDVR0jBBgwFoAU\n" \
"ebRZ5nu25eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAC\n" \
"hhZodHRwOi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcG\n" \
"A1UdHwQgMB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcN\n" \
"AQELBQADggIBAE7iiV0KAxyQOND1H/lxXPjDj7I3iHpvsCUf7b632IYGjukJhM1y\n" \
"v4Hz/MrPU0jtvfZpQtSlET41yBOykh0FX+ou1Nj4ScOt9ZmWnO8m2OG0JAtIIE38\n" \
"01S0qcYhyOE2G/93ZCkXufBL713qzXnQv5C/viOykNpKqUgxdKlEC+Hi9i2DcaR1\n" \
"e9KUwQUZRhy5j/PEdEglKg3l9dtD4tuTm7kZtB8v32oOjzHTYw+7KdzdZiw/sBtn\n" \
"UfhBPORNuay4pJxmY/WrhSMdzFO2q3Gu3MUBcdo27goYKjL9CTF8j/Zz55yctUoV\n" \
"aneCWs/ajUX+HypkBTA+c8LGDLnWO2NKq0YD/pnARkAnYGPfUDoHR9gVSp/qRx+Z\n" \
"WghiDLZsMwhN1zjtSC0uBWiugF3vTNzYIEFfaPG7Ws3jDrAMMYebQ95JQ+HIBD/R\n" \
"PBuHRTBpqKlyDnkSHDHYPiNX3adPoPAcgdF3H2/W0rmoswMWgTlLn1Wu0mrks7/q\n" \
"pdWfS6PJ1jty80r2VKsM/Dj3YIDfbjXKdaFU5C+8bhfJGqU3taKauuz0wHVGT3eo\n" \
"6FlWkWYtbt4pgdamlwVeZEW+LM7qZEJEsMNPrfC03APKmZsJgpWCDWOKZvkZcvjV\n" \
"uYkQ4omYCTX5ohy+knMjdOmdH9c7SpqEWBDC86fiNex+O0XOMEZSa8DA\n" \
"-----END CERTIFICATE-----";


const int door_open_sec = 30;
const int reader_led_pin = 35;
const int reader_24_32_switch = 34;
const int reader_buzzer = 32;

const String dump_keys_request = "https://rfid-prod.shop.thebodgery.org/secure/dump_active_tags";
const char* check_key_request = "https://rfid-prod.shop.thebodgery.org/entry/";
//const String dump_keys_request = "https://rfid-dev.shop.thebodgery.org/secure/dump_active_tags";
//const char* check_key_request = "https://rfid-dev.shop.thebodgery.org/entry/";

// Time to rebuild cache
const unsigned long cache_rebuild_time_ms = 60 * 60 * 1000;

// We're going to store a lot of keys, and this is our main thing, so 
// make a lot of room.
const int dict_size = 21600;
DynamicJsonDocument key_cache( dict_size );
unsigned long ms_since_cache = 0;
const unsigned int CACHE_BUILD_TRIES = 3;

// Pins for Wiegand reads. These must be able to handle interrupts. All GPIO
// pins on the ESP32 can handle it, but that may be different on other chips
const int DATA0 = 22;
const int DATA1 = 23;
const int WIEGAND_BIT_LENGTH = Wiegand::LENGTH_ANY;
//const int WIEGAND_BIT_LENGTH = 8;
Wiegand wiegand;

// Door state management
const int door_open_time_ms = 30 * 1000;
bool is_door_open = false;
unsigned long door_opened_at = 0;
const int DOOR_PIN = 13;


void setup()
{
    Serial.begin( 115200 );
    init_wifi();
    rebuild_cache();
    init_wiegand();

    pinMode( DOOR_PIN, OUTPUT );
    digitalWrite( DOOR_PIN, LOW );
}

void loop()
{
    check_wiegand();
    check_serial_commands();
    check_cache_build_time();
    check_door_status();
}


void init_wifi()
{
    Serial.print( "Connecting to " );
    Serial.print( ssid );
    Serial.print( " " );

    WiFi.setHostname( hostname );
    WiFi.begin( ssid, psk );
    reconnect_wifi();
}

void init_wiegand()
{
    Serial.println( "[WIEGAND.INIT] Startup Wiegand" );
    Serial.flush();

    pinMode( reader_24_32_switch, OUTPUT );
    digitalWrite( reader_24_32_switch, LOW );
    pinMode( reader_led_pin, OUTPUT );
    digitalWrite( reader_led_pin, LOW );
    pinMode( reader_buzzer, OUTPUT );
    digitalWrite( reader_buzzer, LOW );

    pinMode( DATA0, INPUT );
    pinMode( DATA1, INPUT );

    wiegand.onReceive( wiegand_receive, "[WIEGAND] Card read: " );
    wiegand.onReceiveError( wiegand_error, "[WIEGAND] Card read error: " );
    wiegand.onStateChange( wiegand_state_change, "[WIEGAND] State changed: " );
    wiegand.begin( WIEGAND_BIT_LENGTH, true );


    Serial.println( "[WIEGAND.INIT] Wiegand has started" );
    Serial.flush();
}

bool is_wifi_up()
{
    if( WiFi.status() != WL_CONNECTED ) {
        Serial.println( "WiFi is down" );
        return false;
    }

    return true;
}

void reconnect_wifi()
{
    if( is_wifi_up() ) {
        Serial.println( "WiFi is up, forcing disconnect" );
        WiFi.disconnect();
    }

    Serial.println( "Reconnecting WiFi" );
    WiFi.reconnect();

    while( ! is_wifi_up() ) {
        delay( 1000 );
        Serial.print( "." );
        Serial.flush();
    }

    IPAddress gateway = WiFi.gatewayIP();

    Serial.println( "Connected!" );
    Serial.print( "IP: " );
    Serial.println( WiFi.localIP() );
    Serial.print( "Default Gateway: " );
    Serial.println( gateway );
    Serial.flush();
}

void check_serial_commands()
{
    if( Serial.available() ) {
        String cmd = Serial.readString();
        cmd.trim();
        handle_serial_command( cmd );
    }
}

void handle_serial_command( String cmd )
{
    if( cmd.equals( "open" ) ) {
        Serial.println( "[CMD] Opening door by serial command" );
        Serial.flush();
        open_door();
    }
    else if( cmd.equals( "stats" ) ) {
        Serial.print( "[STATS] Bodgery Doorbot v" );
        Serial.println( version );
        Serial.print( "[STATS] IP: " );
        Serial.println( WiFi.localIP() );
        Serial.print( "[STATS] Hostname: " );
        Serial.println( hostname );
        Serial.print( "[STATS] MAC Address: " );
        Serial.println( WiFi.macAddress() );
        Serial.print( "[STATS] Dump keys URL: " );
        Serial.println( dump_keys_request );
        Serial.print( "[STATS] Check keys URL: " );
        Serial.println( check_key_request );
        Serial.print( "[STATS] Keys in cache: " );
        Serial.println( key_cache.size() );
    }
    else if( cmd.equals( "newcache" ) ) {
        Serial.println( "[CMD] Manually starting cache rebuild" );
        rebuild_cache();
    }
    else if( cmd.startsWith( "check" ) ) {
        if( int space_index = cmd.indexOf( ' ' ) ) {
            String fob_id = cmd.substring( space_index + 1 );
            Serial.print( "[CHECK.CMD] Checking fob ID <" );
            Serial.print( fob_id );
            Serial.println( ">" );
            Serial.flush();

            bool found_tag = key_cache.containsKey( fob_id );
            Serial.print( "[CHECK.CMD] Tag in cache: " );
            Serial.println( found_tag );
            Serial.flush();

            bool is_remote = check_tag_remote( fob_id );
            Serial.print( "[CHECK.CMD] Tag on remote server: " );
            Serial.println( is_remote );
            Serial.flush();
        }
        else {
            Serial.println( "[CMD] 'check' command needs a parameter" );
            Serial.flush();
        }
    }
    else if( cmd.equals( "help" ) ) {
        Serial.println( "[HELP] Commands:" );
        Serial.println( "[HELP] check <ID> - Check if a keyfob is valid" );
        Serial.println( "[HELP] newcache - Rebuild the cache" );
        Serial.println( "[HELP] open - Open the door" );
        Serial.println( "[HELP] stats - Dump info about this doorbot" );
    }
    else {
        Serial.print( "[CMD] Unrecognized command: {" );
        Serial.print( cmd );
        Serial.println( "}" );
        Serial.flush();
    }
}

void check_wiegand()
{
    wiegand.flush();

    wiegand.setPin0State( digitalRead( DATA0 ) );
    wiegand.setPin1State( digitalRead( DATA1 ) );
}

void check_tag( String tag )
{
    Serial.print( "[CHECK] Checking tag: " );
    Serial.println( tag );

    bool found_tag = key_cache.containsKey( tag );
    Serial.print( "[CHECK] Tag in cache: " );
    Serial.println( found_tag );
    Serial.flush();

    if( found_tag ) {
        Serial.println( "[CHECK.CACHE] Tag valid in local cache" );
        Serial.flush();
        do_success();
        log_tag_remote( tag );
    }
    else if( check_tag_remote( tag ) ) {
        do_success();
    }
    else {
        Serial.println( "[CHECK] Tag is not valid" );
        Serial.flush();
        do_fail();
    }
}

bool check_tag_remote( String tag )
{
    Serial.print( "[CHECK.REMOTE] Checking if tag is valid: " );
    Serial.println( tag );
    Serial.flush();

    HTTPClient http;
    String request = "";
    request.concat( check_key_request );
    request.concat( tag );
    request.concat( "/" );
    request.concat( location );

    Serial.print( "[CHECK.REMOTE] Sending request to: " );
    Serial.println( request );
    Serial.flush();

    http.begin( request.c_str(), root_ca );
    http.setAuthorization( auth_user, auth_passwd );
    int status = http.GET();

    bool result = false;
    if( HTTP_CODE_OK == status ) {
        Serial.println( "[CHECK.REMOTE] Key is OK" );
        Serial.flush();
        result = true;
    }
    else {
        Serial.print( "[CHECK.REMOTE] Could not verify key: " );
        Serial.println( status );
        Serial.flush();
    }

    http.end();
    return result;
}

bool log_tag_remote( String tag )
{
    Serial.println( "[ENTRY.LOG] Logging entry" );
    Serial.flush();
    check_tag_remote( tag );
    return true;
}

void open_door()
{
    Serial.println( "[DOOR] Instructed to open" );
    Serial.flush();

    if( is_door_open ) {
        Serial.println( "[DOOR] Door already open, ignoring" );
        Serial.flush();
    }
    else {
        door_opened_at = millis();
        is_door_open = true;
        digitalWrite( DOOR_PIN, HIGH );

        Serial.println( "[DOOR] Opening door" );
        Serial.flush();
    }
}

void close_door()
{
    Serial.println( "[DOOR] Closing door" );
    Serial.flush();

    is_door_open = false;
    digitalWrite( DOOR_PIN, LOW );
}

void check_door_status()
{
    if( is_door_open
        && ( millis() >= door_open_time_ms + door_opened_at )
    ) {
        Serial.println( "[DOOR] Door open time has elapsed, closing" );
        close_door();
    }
}

void do_success()
{
    open_door();
}

void do_fail()
{
    // Do nothing, for now
}

void wiegand_pin_state_change()
{
    wiegand.setPin0State( digitalRead( DATA0 ) );
    wiegand.setPin1State( digitalRead( DATA1 ) );
}

void wiegand_state_change( bool plugged, const char* message )
{
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
    Serial.flush();
}

void wiegand_receive(
    uint8_t* data
    ,uint8_t bits
    ,const char* message
)
{
    uint8_t bytes = (bits+7)/8;

    Serial.print(message);
    Serial.print( "{ length: " );
    Serial.print( bytes );
    Serial.print( " } { bits: " );
    Serial.print( bits );
    Serial.print( " }" );
    Serial.flush();

    unsigned long long_data = 0;
    for( int i = 0; i < bytes; i++ ) {
        long_data <<= 8;
        long_data |= data[i];
    }
    String str_data_no_prefix = String( long_data );

    String str_data = "";
    // Original database stores numbers with 0's prefix padding to a length of 
    // 10, so correct for this
    while( (str_data_no_prefix.length() + str_data.length()) < 10 ) {
        str_data.concat( "0" );
    }
    str_data.concat( str_data_no_prefix );

    Serial.print( " { formatted: " );
    Serial.print( str_data );
    Serial.println( " }" );
    Serial.flush();

    check_tag( str_data );
}

void wiegand_error(
    Wiegand::DataError error
    ,uint8_t* rawData
    ,uint8_t rawBits
    ,const char* message
)
{
    Serial.print( "[WIEGAND.ERROR] " );
    Serial.print(message);
    Serial.print(Wiegand::DataErrorStr(error));
    Serial.print(" - Raw data: ");
    Serial.print(rawBits);
    Serial.print("bits / ");

    //Print value in HEX
    uint8_t bytes = (rawBits+7)/8;
    for( int i = 0; i < bytes; i++ ) {
        Serial.print(rawData[i] >> 4, 16);
        Serial.print(rawData[i] & 0xF, 16);
    }

    Serial.println();
    Serial.flush();
}

void check_cache_build_time()
{
    unsigned long ms_since = millis() - ms_since_cache;
    if( cache_rebuild_time_ms <= ms_since ) {
        Serial.print( "[CACHE] " );
        Serial.print( ms_since );
        Serial.println( "ms has passed since cache built, rebuilding" );
        rebuild_cache();
    }
}

void rebuild_cache()
{
    unsigned long mem_before = ESP.getFreeHeap();
    Serial.print( "[CACHE] Heap free before rebuilding cache: " );
    Serial.println( mem_before );

    rebuild_cache( CACHE_BUILD_TRIES );

    unsigned long mem_after = ESP.getFreeHeap();
    long extra_usage = mem_before - mem_after;
    Serial.print( "[CACHE] Heap free after rebuilding cache: " );
    Serial.print( mem_after );
    Serial.print( " (" );
    Serial.print( extra_usage );
    Serial.println( " more bytes)" );
}

void rebuild_cache(
    unsigned int tries_left
)
{
    if( 0 == tries_left ) {
        // Give up. Have to set time since cache here, or else it will try 
        // to run the cache rebuild again on the next loop.
        ms_since_cache = millis();
        return;
    }

    HTTPClient http;

    Serial.println( "[CACHE] Rebuild cached keys" );
    http.begin( dump_keys_request, root_ca );
    http.addHeader( "Accept", "application/json" );
    http.setAuthorization( auth_user, auth_passwd );
    int status = http.GET();

    if( HTTP_CODE_OK == status ) {
        Serial.println( "[CACHE] Fetched new key database" );
        Serial.flush();
        String body = http.getString();
        int len = body.length();

        Serial.print( "[CACHE] Received " );
        Serial.print( len );
        Serial.println( " bytes" );
        Serial.println( "[CACHE] Rebuilding dictionary" );
        Serial.flush();

        key_cache.clear();
        DeserializationError error = deserializeJson( key_cache, body );
        if( error ) {
            Serial.print( "[CACHE] Deserialization error: " );
            Serial.println( error.f_str() );
            rebuild_cache( tries_left - 1 );
            return;
        }
        Serial.print( "[CACHE] Processed " );
        Serial.print( key_cache.size() );
        Serial.println( " keys" );
        Serial.flush();

        // Reset time since rebuild
        ms_since_cache = millis();

        Serial.println( "[CACHE] Cache rebuild successful" );
        Serial.flush();
    }
    else {
        Serial.print( "[CACHE] Error fetching new key database: " );
        Serial.println( status );
        Serial.print( "[CACHE] HTTP status: " );
        Serial.println( http.getString() );
        Serial.print( "[CACHE] Trying cache rebuild again, tries left: " );
        Serial.println( tries_left - 1 );
        Serial.flush();

        rebuild_cache( tries_left - 1 );
    }
}
