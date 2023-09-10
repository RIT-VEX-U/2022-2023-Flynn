#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>
#include "vex.h"

enum LogLevel { DEBUG, NOTICE, WARNING, ERROR, CRITICAL, TIME};

class Logger {
private:
    const std::string filename;
    vex::brain::sdcard sd;
    void write_level(LogLevel l);

public:
    const int MAX_FORMAT_LEN = 512;
    explicit Logger(const std::string &filename);

    // No copying or reassigning bc then who owns the file?
    Logger(const Logger &l) = delete;
    Logger &operator=(const Logger &l) = delete;


    void flush();

    // Logging

    void Log(const std::string &s);
    void Log(LogLevel level, const std::string &s);

    void Logln(const std::string &s);
    void Logln(LogLevel level, const std::string &s);

    void Logf(const char *fmt, ...);
    void Logf(LogLevel level, const char *fmt, ...);
};
