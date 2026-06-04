#pragma once
#include <Arduino.h>

void uartInit();

void uartTask();

void sendToSTM32(String msg);