#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <vector>

using namespace std;

/* Congestion controller interface */

class Controller
{
private:
    bool debug_;

    // Additive increase
    int ai = 1;

    // Multiplicative decrease
    double md = 0.5;

    // Current window size
    unsigned int cur_ws = 10;

public:
    Controller(const bool debug);

    unsigned int window_size();

    void datagram_was_sent(const uint64_t sequence_number,
                const uint64_t send_timestamp);

    void ack_received(const uint64_t sequence_number_acked,
                const uint64_t send_timestamp_acked,
                const uint64_t recv_timestamp_acked,
                const uint64_t timestamp_ack_received);

    unsigned int timeout_ms();
};

#endif
