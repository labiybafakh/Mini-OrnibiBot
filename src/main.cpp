#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

extern "C" {
    #include "zenoh-pico.h"
}

// WiFi-specific parameters
#define SSID "SSID"
#define PASS "PASSWORD"

// Zenoh-specific parameters
#define MODE "client"
#define URI "/rt/cmd_vel"

// Measurement specific parameters
#define X_SCALING_FACTOR 100.0
#define X_MAX_VALUE 0.20
#define X_MIN_VALUE -0.20

#define Y_SCALING_FACTOR 10.0
#define Y_MAX_VALUE 2.80
#define Y_MIN_VALUE -2.80

MPU6050 mpu(Wire);
double offset_x = 0.0;
double offset_y = 0.0;

zn_session_t *s = NULL;
zn_reskey_t *reskey = NULL;

void setup(void)
{
    // Set WiFi in STA mode and trigger attachment
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED)
        delay(1000);

    // Initialize MPU6050
    Wire.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true);
    mpu.update();
    offset_x = mpu.getAccAngleX();
    offset_y = mpu.getAccAngleY();

    // Initialize Zenoh Session and other parameters
    zn_properties_t *config = zn_config_default();
    zn_properties_insert(config, ZN_CONFIG_MODE_KEY, z_string_make(MODE));

    s = zn_open(config);
    if (s == NULL)
        return;

    znp_start_read_task(s);
    znp_start_lease_task(s);

    unsigned long rid = zn_declare_resource(s, zn_rname(URI));
    reskey = (zn_reskey_t*)malloc(sizeof(zn_reskey_t));
    *reskey = zn_rid(rid);

    delay(1000);
}

void loop()
{
    delay(20);
    mpu.update();

    double linear_x = (mpu.getAccAngleX() - offset_x) / X_SCALING_FACTOR;
    linear_x = min(max(linear_x, X_MIN_VALUE), X_MAX_VALUE);
    if (linear_x < 0.10 && linear_x > -0.10)
            linear_x = 0;

    double linear_y = (mpu.getAccAngleY() - offset_y) / Y_SCALING_FACTOR;
    linear_y = min(max(linear_y, Y_MIN_VALUE), Y_MAX_VALUE);
    if (linear_y < 0.5 && linear_y > -0.5)
            linear_y = 0;

    Twist measure;
    measure.linear.x = linear_x;
    measure.linear.y = 0.0;
    measure.linear.z = 0.0;
    measure.angular.x = 0.0;
    measure.angular.y = 0.0;
    measure.angular.z = linear_y;

    uint8_t twist_serialized_size = 4 + sizeof(double) * 6;
    char buf[twist_serialized_size];
    serialize_twist(&measure, buf);

    if (s == NULL || reskey == NULL)
        return;

    zn_write(s, *reskey, (const uint8_t *)buf, twist_serialized_size);
}