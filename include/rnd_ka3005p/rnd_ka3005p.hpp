#ifndef __RND_KA3005P_H__
#define __RND_KA3005P_H__

#include <chrono>
#include "serial/serial.h"
#include <mutex>

enum RndKa3005pMode
{
    CONSTANT_CURRENT = 0,
    CONSTANT_VOLTAGE
};

struct RndKa3005pStatusMsg
{
    enum RndKa3005pMode mode;
    bool output_on;
    bool over_voltage_protection_on;
    bool over_current_protetion_on;
};

class RndKa3005p
{
private:
    serial::Serial dev_;

    std::chrono::time_point<std::chrono::steady_clock> last_command_time_;
    float min_time_between_commands_ms_;

    void sleep_until_next_command_();
    void write_command_(std::string cmd);
    float write_and_read_float_w_retry_(std::string cmd, int retval_len, int retry_times = 3);

    std::mutex device_mutex_;

public:
    RndKa3005p();

    void open(std::string port, float min_time_between_commands_ms = 30);

    void set_output(bool is_on);

    void set_voltage(float voltage);
    float get_voltage_setpoint();
    void set_current(float current);
    float get_current_setpoint();

    float get_voltage();
    float get_current();

    std::string get_identification();
    struct RndKa3005pStatusMsg get_status();
};

#endif // __RND_KA3005P_H__