#include <sstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <assert.h>
#include <bitset>
#include "rnd_ka3005p/rnd_ka3005p.hpp"

RndKa3005p::RndKa3005p()
    : min_time_between_commands_ms_(0),
      last_command_time_(std::chrono::steady_clock::now())
{
}

void RndKa3005p::open(std::string port, float min_time_between_commands_ms)
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    dev_.setPort(port);
    dev_.setBaudrate(9600);

    dev_.setTimeout(1000, 1000, 0, 1000, 0);

    dev_.open();

    min_time_between_commands_ms_ = min_time_between_commands_ms;
    last_command_time_ = std::chrono::steady_clock::now();
}

void RndKa3005p::sleep_until_next_command_()
{
    auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_command_time_);
    float time_since_last_command_ms = time_since_last_command.count();
    if (time_since_last_command_ms < min_time_between_commands_ms_)
    {
        std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1, 1000>>(min_time_between_commands_ms_ - time_since_last_command_ms));
    }
}

void RndKa3005p::write_command_(std::string cmd)
{
    sleep_until_next_command_();
    dev_.write(cmd);
    last_command_time_ = std::chrono::steady_clock::now();
}

float RndKa3005p::write_and_read_float_w_retry_(std::string cmd, int retval_len, int retry_times)
{
    retry_times--;

    for (int retry_i = 0; retry_i < retry_times; retry_i++)
    {
        try
        {
            write_command_(cmd);
            std::string results = dev_.read(retval_len);
            return std::stof(results);
        }
        catch (...)
        {
            continue;
        }
    }

    // Last time will throw the exception in case
    // it doesn't succeed
    write_command_(cmd);
    std::string results = dev_.read(retval_len);
    return std::stof(results);
}

void RndKa3005p::set_output(bool is_on)
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    write_command_(is_on ? "OUT1" : "OUT0");
}

void RndKa3005p::set_voltage(float voltage)
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << voltage;
    std::string voltage_text = ss.str();
    std::string command = "VSET1:" + voltage_text;
    write_command_(command);
}
float RndKa3005p::get_voltage_setpoint()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    return write_and_read_float_w_retry_("VSET1?", 5);
}
void RndKa3005p::set_current(float current)
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << current;
    std::string current_text = ss.str();
    std::string command = "ISET1:" + current_text;
    write_command_(command);
}
float RndKa3005p::get_current_setpoint()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    return write_and_read_float_w_retry_("ISET1?", 5);
}
float RndKa3005p::get_voltage()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    return write_and_read_float_w_retry_("VOUT1?", 5);
}
float RndKa3005p::get_current()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    return write_and_read_float_w_retry_("IOUT1?", 5);
}

std::string RndKa3005p::get_identification()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    write_command_("*IDN?");
    std::string results = dev_.readline();
    return results;
}

struct RndKa3005pStatusMsg RndKa3005p::get_status()
{
    const std::lock_guard<std::mutex> lock(device_mutex_);

    write_command_("STATUS?");
    uint8_t status_byte = 0;
    assert(dev_.read(&status_byte, 1) == 1);

    std::bitset<8> status_bits(status_byte);

    struct RndKa3005pStatusMsg retval;
    retval.output_on = status_bits.test(6);
    retval.mode = status_bits.test(0) ? CONSTANT_VOLTAGE : CONSTANT_CURRENT;
    retval.over_current_protetion_on = status_bits.test(5);
    retval.over_voltage_protection_on = status_bits.test(7);

    return retval;
}