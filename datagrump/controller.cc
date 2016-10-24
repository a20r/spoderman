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
    packetToId(),
    packetToSendTime(),
    distribution()
{   
    reset_weights();
    Exp3();
}

void Controller::Exp3() {
    // Compute the probabilities associated with drawing each arm.
    compute_probabilities();
    // Randomly draw an arm according to the computed distribution.
    std::size_t arm = distribution(gen);
    //std::cout << "Randomly generated arm " << arm << std::endl;
    cur_arm = arm;

    // Map the drawn arm to the congestion window.
    cur_ws = arm_to_congestion_window(arm);
    //std::cout << "Corresponding congestion window " << cur_ws << std::endl;

    // Replan after sending the packet with the following sequence number.
    replan += cur_ws;

    // "Tag" the packet sequence number with this arm so that the reward is
    // calculated when the ack for the corresponding packet sequence number
    // is received.
    auto armCongestionWindowPair = std::make_pair(cur_arm, cur_ws);
    packetToId[replan] = armCongestionWindowPair;
}

void Controller::reset_weights() {
    for (std::size_t i = 0; i < weights.size(); ++i) {
        weights[i] = 1;
    }
}

void Controller::reset_weights_low() {
    for (std::size_t i = 0; i < weights.size(); ++i) {
        weights[i] = 1.0/(i + 1);
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
    // DEBUGGING
    // {
    //     cerr << "At time " << timestamp_ms()
    //         << " window size is " << cur_ws << endl;
    // }
    return cur_ws;
}

void Controller::datagram_was_sent(
		/* of the sent datagram */
        const uint64_t sequence_number,
        /* in milliseconds */
		const uint64_t send_timestamp)
{
    // Should we draw a new arm?
    if (sequence_number == replan) {
        std::cout << std::endl << "Replanning at sequence number: " 
                  << sequence_number << std::endl;
        // Mark the time that the packet was sent at so that we can
        // compute the reward afterwards.
        packetToSendTime[sequence_number] = send_timestamp;
        Exp3();
    }

    // if (replan <= sequence_number_acked) {
    //     compute_probabilities();
    //     std::size_t arm = distribution(gen);
    //     std::cout << "Randomly generated arm " << arm << std::endl;
    //     cur_arm = arm;
    //     cur_ws = arm_to_congestion_window(arm);
    //     // std::cout << "Total reward thus far " << totalReward << std::endl;
    //     std::cout << "Corresponding congestion window " << cur_ws << std::endl;
    //     replan = sequence_number_acked + cur_ws;
    // }

    DEBUGGING
    {
        cerr << "At time " << send_timestamp
            << " sent datagram " << sequence_number << endl;
    }
}


void Controller::DistributeReward(std::size_t arm, double rate) {
    std::vector<double> probabilities = distribution.probabilities();
    for (std::size_t i = 0; i < K; ++i) {
        std::size_t distance = abs(arm - i);

        double reward = rate / probabilities[i];
        double thisReward = reward / exp(distance);
        double multiplicativeFactor = gamma * thisReward / K;
        weights[i] *= exp(multiplicativeFactor); 
    }
}
// RECALCULATION OF CWND SHOULD OCCUR IN THE SENDING
// CALCULATION OF REWARD IS NOT CORRECT
// THROUGHPUT = CWND / RTT
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
    // uint64_t interArrivalTime = max(recv_timestamp_acked - last_ts, uint64_t(1));
    // last_ts = recv_timestamp_acked;

    ++numPackets;
    // Should we compute the reward?
    auto it = packetToId.find(sequence_number_acked);

    if (it != packetToId.end()) {
        // Compute the reward for this arm.
        auto armWindowPair = it->second;
        std::size_t arm = armWindowPair.first;
        std::size_t congestionWindow = armWindowPair.second;

        // Reward is computed as the rate of packet acknowledgements in the time
        // frame between the sending of the last packet and the receiving of the last
        // packet associated with the congestion window.
        //double timeFrame = timestamp_ack_received - packetToSendTime[sequence_number_acked];
        double timeFrame = timestamp_ack_received - last_ts;
        double RATE_THRESHOLD = 0.1;
        double rate = congestionWindow / timeFrame;

        //double reward = (rate - RATE_THRESHOLD) / probabilities[arm];

        //double multiplicativeFactor = gamma * reward / K;
        //weights[arm] *= exp(multiplicativeFactor); 

        if (rate < 0.05) {
            std::cout << "\n\nExtremely low rate: " << rate << " cwnd = " << cur_ws << std::endl << std::endl;
            reset_weights_low();
        }

        //DistributeReward(arm, 10*(rate - RATE_THRESHOLD));
        DistributeReward(arm, rate - RATE_THRESHOLD);

        std::cout << "Time frame: " << timeFrame << std::endl;
        std::cout << "Rate: " << rate << std::endl;
        //std::cout << "probability of this arm: " << probabilities[arm] << std::endl;
        //std::cout << "reward: " << reward << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        //std::cout << "Multiplicative factor: " << multiplicativeFactor << std::endl;
        std::cout << "weights: " << weights[arm] << std::endl;

        //last_ts = recv_timestamp_acked;
        last_ts = timestamp_ack_received;
    }

    if (numPackets % 10000 == 0)
    {
        std::cout << "\nResetting weights\n" << std::endl;
        reset_weights();
    }

    // // To-do: consider rescaling the "reward" based on what happened previously.
    // ++numPackets;
    // auto probabilities = distribution.probabilities();
    // std::size_t arm = packetToArm[sequence_number_acked];

    // //uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    // //std::cout << "rrt: " << rtt << std::endl;
    // float reward = 0;
    // if (interArrivalTime < 100) {
    //     reward = (1.0 / (interArrivalTime*cur_ws)) / (probabilities[arm]);
    // } else {
    //     std::cout << "interarrivalTime " << interArrivalTime << std::endl;
    //     cur_ws = 1;
    //     cur_arm = 0;
    //     return;
    // }

    // weights[arm] *= exp(gamma * reward / K);
    // totalReward += reward;
    // std::cout << "probabilities: " << probabilities[arm] << std::endl;
    // std::cout << "reward: " << reward << std::endl;
    // std::cout << "gamma: " << gamma << std::endl;
    // std::cout << weights[arm] << std::endl;

    // if (replan <= sequence_number_acked) {
    //     compute_probabilities();
    //     std::size_t arm = distribution(gen);
    //     std::cout << "Randomly generated arm " << arm << std::endl;
    //     cur_arm = arm;
    //     cur_ws = arm_to_congestion_window(arm);
    //     // std::cout << "Total reward thus far " << totalReward << std::endl;
    //     std::cout << "Corresponding congestion window " << cur_ws << std::endl;
    //     replan = sequence_number_acked + cur_ws;
    // }


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
