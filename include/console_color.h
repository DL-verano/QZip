#pragma once

#include <iostream>

#if defined(WIN32)
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

inline std::ostream& blue(std::ostream& s) {
#if defined(WIN32)
    HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hStdout, FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
#else
    s << "\e[0;34m";
#endif
    return s;
}

inline std::ostream& red(std::ostream& s) {
#if defined(WIN32)
    HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_INTENSITY);
#else
    s << "\e[0;31m";
#endif
    return s;
}

inline std::ostream& green(std::ostream& s) {
#if defined(WIN32)
    HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
#else
    s << "\e[0;32m";
#endif
    return s;
}

inline std::ostream& yellow(std::ostream& s) {
#if defined(WIN32)
    HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY);
#else
    s << "\e[0;33m";
#endif
    return s;
}

inline std::ostream& white(std::ostream& s) {
#if defined(WIN32)
    HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
    s << "\e[0;39m";
#endif
    return s;
}
