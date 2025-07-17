#ifndef LOGGER_H
#define LOGGER_H

enum LogLevel {
  DEBUG,
  INFO,
  WARN,
  ERROR
};

class Logger {
  public:
    Logger();
    void init(long baudRate);
    void setLogLevel(LogLevel level);
    void write(LogLevel level, const char* message);

  private:
    LogLevel currentLevel;
    const char* getLevelString(LogLevel level);
};

#endif