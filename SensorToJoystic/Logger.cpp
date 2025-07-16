#include "Logger.h"
#include <Arduino.h>

Logger::Logger() : currentLevel(INFO) {}

void Logger::init(long baudRate) {
  Serial.begin(baudRate);
}

void Logger::setLogLevel(LogLevel level) {
  currentLevel = level;
}

void Logger::write(LogLevel level, const char* message) {
  if (level >= currentLevel) {
    unsigned long timestamp = millis();
    Serial.print("[");
    Serial.print(timestamp);
    Serial.print("] ");
    Serial.print(getLevelString(level));
    Serial.print(": ");
    Serial.println(message);
  }
}

const char* Logger::getLevelString(LogLevel level) {
  switch (level) {
    case DEBUG: return "DEBUG";
    case INFO:  return "INFO";
    case WARN:  return "WARN";
    case ERROR: return "ERROR";
    default:    return "UNKNOWN";
  }
}