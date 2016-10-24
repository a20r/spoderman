#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <random>

using namespace std;
using ts_t = uint64_t;

/* Congestion controller interface */

class Controller
{
private:
    bool debug_;

    // Current window size
    int cur_ws = 10;

    double link_rate = 0.0;
    int num_acks = 0;
    ts_t tick_length = 20;
    ts_t start_time = 0;
    ts_t delay_target = 100;
    ts_t forecast_length = 160;
    int vol = 200;
    default_random_engine gen;
    int n_rates = 256;
    double *rates;
    double *probs;
    int in_transit = 0;
    double min_rate = 1000 / 256.0;

public:
    Controller(const bool debug);

    unsigned int window_size();

    void datagram_was_sent(const uint64_t sequence_number,
                const uint64_t send_timestamp);

    void evolve_rates(double *rs, double secs);
    void evolve(double time);

    void ack_received(const uint64_t sequence_number_acked,
                const uint64_t send_timestamp_acked,
                const uint64_t recv_timestamp_acked,
                const uint64_t timestamp_ack_received);

    unsigned int timeout_ms();
};

#endif
