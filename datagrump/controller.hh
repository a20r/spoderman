#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <deque>
#include <random>
#include <memory>
#include <unordered_map>

using namespace std;

/* Congestion controller interface */

class Controller
{
private:
    // Maps a multi-armed bandit arm to a randomly drawn congestion window in 
    // congestion window in the interval
    // [arm * DELTA_WINDOW, arm * DELTA_WINDOW + DELTA_WINDOW]
    void reset_weights();
    std::size_t arm_to_congestion_window(uint64_t arm);
    void compute_probabilities();

    static const std::size_t MAX_WINDOW = 30;
    static const std::size_t DELTA_WINDOW = 2;
    static constexpr float G = 10000;

    bool debug_;
    bool is_window_set;

    // Current window size
    std::size_t cur_ws = 10;

    std::size_t numPackets = 0;

    // Replan sequence after packet ID.
    uint64_t replan = 1;

    // Current arm.
    std::size_t cur_arm = 2;

    // Number of arms.
    std::size_t K;

    float gamma;

    // List of weights for each arm.
    std::vector<float> weights;

    std::random_device rd;
    std::mt19937 gen;

    // Map of each packet identifier to corresponding "arm."
    std::unordered_map<uint64_t, std::size_t> packetToArm;
    std::discrete_distribution<> distribution;

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
