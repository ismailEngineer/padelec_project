#pragma once

#include <Arduino.h>

struct TelemetryData
{
    float speed;
    float totalMileage;
    float lastTripDistance;

    int batteryLevel;

    bool locked;

    String drivingMode;
};

extern TelemetryData telemetry;

String buildTelemetryJson();