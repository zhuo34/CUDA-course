#include "timer.h"
#include <iostream>

using namespace std::chrono_literals;

void Timer::start() {
    if (!is_started)
        is_started = true;
    is_running = true;
    start_ts = system_now();
    lap_ts = start_ts;
}

double Timer::stop() {
    if (!is_running)
        return now();
    auto t = system_now();
    pause_time += get_duration(start_ts, t);
    lap_pause_time += get_duration(lap_ts, t);
    is_running = false;
    return now();
}

double Timer::end() {
    auto t = stop();
    reset();
    return t;
}

void Timer::reset() {
    if (!is_started || is_running)
        return;
    is_running = false;
    is_started = false;
    pause_time = duration::zero();
    lap_pause_time = duration::zero();
    lap_times.clear();
    _min_lap_time = duration::max();
    _max_lap_time = duration::zero();
    _lap_cnt = _min_lap = _max_lap = 0;
}

double Timer::now() const {
    if (!is_running)
        return micro2sec(pause_time);
    auto t = system_now();
    return micro2sec(get_duration(start_ts, t) + pause_time);
}

double Timer::lap() {
    if (!is_running)
        return lap_now();
    auto d = lap_pause_time + get_duration(lap_ts, system_now());
    lap_times.push_back(d);
    _lap_cnt ++;
    if (d < _min_lap_time) {
        _min_lap_time = d;
        _min_lap = _lap_cnt;
    }
    if (d > _max_lap_time) {
        _max_lap_time = d;
        _max_lap = _lap_cnt;
    }
    lap_pause_time = duration::zero();
    lap_ts = system_now();
    return micro2sec(d);
}

double Timer::lap_now() const {
    if (!is_running)
        return micro2sec(lap_pause_time);
    return micro2sec(get_duration(lap_ts, system_now()) + lap_pause_time);
}

int Timer::lap_cnt() const{
    return _lap_cnt;
}

int Timer::max_lap() const{
    return _max_lap;
}

int Timer::min_lap() const{
    return _min_lap;
}

double Timer::max_lap_time() const{
    return micro2sec(_max_lap_time);
}

double Timer::min_lap_time() const{
    return micro2sec(_min_lap_time);
}

time_point Timer::system_now() {
    return std::chrono::system_clock::now();
}

duration Timer::get_duration(const time_point &start, const time_point &end) {
    return std::chrono::duration_cast<duration>(end - start);
}

double Timer::micro2sec(const duration &d) {
    return double(d.count()) * duration::period::num
        / duration::period::den;
}
