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
    reset_weights();
    compute_probabilities();
    cur_arm = floor(cur_ws / DELTA_WINDOW);
}

void Controller::reset_weights() {
    for (std::size_t i = 0; i < weights.size(); ++i) {
        //weights[i] = weights.size() / (i + 1);
        weights[i] = 1;
    }
}

void Controller::compute_probabilities() 
{
    distribution = std::discrete_distribution<>(weights.begin(), weights.end());
    std::vector<double> probabilities = distribution.probabilities();

    std::vector<double> newWeights = probabilities;

    for (auto &prob : newWeights) {
        prob = (1 - gamma) * prob + gamma / K;    
    }

    distribution = std::discrete_distribution<>(newWeights.begin(), newWeights.end());

    for (std::size_t i = 0; i < probabilities.size(); ++i) {
        auto prob = probabilities[i];
        std::cout << "Prob[" << i << "]: " << prob << std::endl;
    }
}

std::size_t Controller::arm_to_congestion_window(std::size_t arm) {
    std::size_t lower = arm * DELTA_WINDOW;
    std::size_t upper = arm * (DELTA_WINDOW) + DELTA_WINDOW;

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

    // RECALCULATION OF CWND SHOULD OCCUR IN THE SENDING
    // CALCULATION OF REWARD IS NOT CORRECT
    // THROUGHPUT = CWND / RTT
    uint64_t interArrivalTime = max(recv_timestamp_acked - last_ts, uint64_t(1));
    last_ts = recv_timestamp_acked;

    // To-do: consider rescaling the "reward" based on what happened previously.
    ++numPackets;
    auto probabilities = distribution.probabilities();
    std::size_t arm = packetToArm[sequence_number_acked];

    //uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    //std::cout << "rrt: " << rtt << std::endl;
    float reward = 0;
    if (interArrivalTime < 100) {
        reward = (1.0 / (interArrivalTime*cur_ws)) / (probabilities[arm]);
    } else {
        std::cout << "interarrivalTime " << interArrivalTime << std::endl;
        //reset_weights();
        cur_ws = 1;
        cur_arm = 0;
        return;
    }

    weights[arm] *= exp(gamma * reward / K);
    totalReward += reward;
    // std::cout << "probabilities: " << probabilities[arm] << std::endl;
    // std::cout << "reward: " << reward << std::endl;
    // std::cout << "gamma: " << gamma << std::endl;
    // std::cout << weights[arm] << std::endl;

    if (replan <= sequence_number_acked) {
        compute_probabilities();
        std::size_t arm = distribution(gen);
        std::cout << "Randomly generated arm " << arm << std::endl;
        cur_arm = arm;
        cur_ws = arm_to_congestion_window(arm);
        // std::cout << "Total reward thus far " << totalReward << std::endl;
        std::cout << "Corresponding congestion window " << cur_ws << std::endl;
        replan = sequence_number_acked + cur_ws;
    }

    if (numPackets % 1000 == 0)
        reset_weights();

    //std::cout << "Num packets received " << ++numPackets << std::endl;
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
