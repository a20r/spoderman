#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include "controller.hh"
#include "timestamp.hh"


#define DEBUGGING if (debug_)

using namespace std;

Controller::Controller(const bool debug)
  : debug_(debug),
    is_window_set(false),
    K(MAX_WINDOW / DELTA_WINDOW), // K denotes the number of arms
    gamma(min(1.0, sqrt( (float(K * log(K))) / ((exp(1) - 1)* G)))),
    weights(K, 1), // Initialize the weights to 1.
    rd(),
    gen(rd()),
    packetToArm(),
    distribution()
{   
    compute_probabilities();
}

void Controller::compute_probabilities() 
{
    distribution = std::discrete_distribution<>(weights.begin(), weights.end());
    auto probabilities = distribution.probabilities();

    for (auto weight : weights) {
        std::cout << weight << std::endl;
    }
    for (auto prob : probabilities) {
        std::cout << prob << std::endl;
    }
}

std::size_t Controller::arm_to_congestion_window(std::size_t arm) {
    std::size_t lower = arm * DELTA_WINDOW;
    std::size_t upper = arm * (DELTA_WINDOW + 1);

    std::uniform_int_distribution<> dis(lower, upper);

    // Generate a number uniformly at random from [lower, upper].
    return static_cast<std::size_t>(max(1, dis(gen)));
}


unsigned int Controller::window_size()
{   
    // if (!is_window_set) {   
    //     compute_probabilities();
    //     std::size_t arm = distribution(gen);
    //     std::cout << "Randomly generated arm " << arm << std::endl;
    //     cur_ws = arm_to_congestion_window(arm);
    //     std::cout << "Corresponding congestion window " << cur_ws << std::endl;
    // }

    DEBUGGING
    {
        cerr << "At time " << timestamp_ms()
            << " window size is " << cur_ws << endl;
    }
    return cur_ws;
}

void Controller::datagram_was_sent(
		/* of the sent datagram */
        const uint64_t sequence_number,
        /* in milliseconds */
		const uint64_t send_timestamp)
{
    packetToArm[sequence_number] = cur_arm;

    DEBUGGING
    {
        cerr << "At time " << send_timestamp
            << " sent datagram " << sequence_number << endl;
    }
}

void Controller::ack_received(
        /* what sequence number was acknowledged */
        const uint64_t sequence_number_acked,
        /* when the acknowledged datagram was sent (sender's clock) */
        const uint64_t send_timestamp_acked,
        /* when the acknowledged datagram was received (receiver's clock)*/
        const uint64_t recv_timestamp_acked,
        /* when the ack was received (by sender) */
        const uint64_t timestamp_ack_received)
{

    // To-do: consider rescaling the "reward" based on what happened previously.
    auto probabilities = distribution.probabilities();
    std::size_t arm = packetToArm[sequence_number_acked];
    float reward = float(recv_timestamp_acked - send_timestamp_acked) / probabilities[arm];
    weights[arm] *= exp(gamma * reward / K);
    std::cout << "reward: " << reward << std::endl;
    std::cout << "gamma: " << gamma << std::endl;
    std::cout << weights[arm] << std::endl;

    if (replan <= sequence_number_acked) {
        compute_probabilities();
        std::size_t arm = distribution(gen);
        std::cout << "Randomly generated arm " << arm << std::endl;
        cur_ws = arm_to_congestion_window(arm);
        std::cout << "Corresponding congestion window " << cur_ws << std::endl;
        replan = sequence_number_acked + cur_ws;
    }

    DEBUGGING
    {
        cerr << "At time " << timestamp_ack_received
        << " received ack for datagram " << sequence_number_acked
        << " (send @ time " << send_timestamp_acked
        << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
        << endl;
    }
}

unsigned int Controller::timeout_ms()
{
    return 1000;
}
