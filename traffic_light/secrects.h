#include <pgmspace.h>

#define SECRET
#define THINGNAME "ESP32-thing"

const char WIFI_SSID[] = "esp";
const char WIFI_PASSWORD[] = "12345678";
const char AWS_IOT_ENDPOINT[] = "a3cr95t92p44bz-ats.iot.ap-southeast-2.amazonaws.com";

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_CERTIFICATION
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
YOUR_CERTIFICATION
-----END CERTIFICATE-----

)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
YOUR_PRIVATE_KEY
-----END RSA PRIVATE KEY-----


)KEY";
