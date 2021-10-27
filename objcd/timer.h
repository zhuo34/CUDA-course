#pragma once

#include <chrono>
#include <vector>
#include <iostream>

typedef std::chrono::system_clock::time_point time_point;
typedef std::chrono::microseconds duration;

class Timer {
private:
    time_point start_ts;
    duration pause_time = duration::zero();
    bool is_running = false;
    bool is_started = false;
    time_point lap_ts;
    duration lap_pause_time = duration::zero();
    std::vector<duration> lap_times;
    int _lap_cnt = 0;
    int _min_lap = 0;
    int _max_lap = 0;
    duration _min_lap_time = duration::max(), _max_lap_time = duration::zero();
public:
    void start();
    void reset();
    double stop();
    double now() const;
    double lap();
    double lap_now() const;
    int lap_cnt() const;
    int max_lap() const;
    int min_lap() const;
    double max_lap_time() const;
    double min_lap_time() const;
public:
    static duration get_duration(const time_point &start, const time_point &end);
    static double micro2sec(const duration &d);
    static time_point system_now();
};