#include <iostream>
#include <cmath>
#include "controller.hh"
#include "timestamp.hh"

#define DEBUGGING if (debug_)

using namespace std;

Controller::Controller(const bool debug)
  : debug_(debug)
{}

unsigned int Controller::window_size()
{
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
    if (counter == 0)
    {
        start_time = timestamp_ack_received;
    }

    if (++counter > 10)
    {
        uint64_t end_time = timestamp_ack_received;
        link_rate = counter / (double) (end_time - start_time);
        counter = 0;
        cout << link_rate << endl;
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
