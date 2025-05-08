#pragma once
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>

class ScopeTimer {
    static int depth;
    bool active_;
    bool stopped_;
    bool oneline_;
    std::string msg_;
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> timepoint;
    timepoint start_;

  public:
    static double difference_millis(timepoint start, timepoint end) { return double(std::chrono::duration<double>(end - start).count()); }
    static double difference_seconds(timepoint start, timepoint end) { return double(std::chrono::duration<double>(end - start).count()); }
    void activate() {
        depth++;
        active_ = true;
        start_ = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < depth; i++) {
            std::cout << "    ";
        }
        if (!oneline_)
            std::cout << "/ " << msg_ << "..." << std::endl;
        else
            std::cout << "- " << msg_ << "...";
    }

    double stop() {
        if (active_ && !stopped_) {
            double s = time();
            auto precision_stack = double(std::cout.precision());
            if (!oneline_) {
                for (int i = 0; i < depth; i++) {
                    std::cout << "    ";
                }
                std::cout << "\\ " << std::setprecision(3) << std::fixed << s << " s" << std::endl;
            } else
                std::cout << " " << std::setprecision(3) << std::fixed << s << " s" << std::endl;
            std::cout << std::setprecision(std::streamsize(precision_stack)) << std::defaultfloat;
            depth--;
            stopped_ = true;
            return s;
        } else
            return 0.0;
    }

    double time() const { return difference_seconds(start_, std::chrono::high_resolution_clock::now()); }

    ScopeTimer(const ScopeTimer &) = delete;
    ScopeTimer(const ScopeTimer &&) = delete;
    ScopeTimer &operator=(const ScopeTimer &) = delete;
    ScopeTimer &operator=(const ScopeTimer &&) = delete;

    ScopeTimer(std::string msg, bool active = true, bool oneline = true) : active_(active), stopped_(false), oneline_(oneline), msg_(std::move(msg)) {
        if (active_)
            activate();
    }

    ~ScopeTimer() { stop(); }
};
