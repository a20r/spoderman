#include <iostream>
#include <cmath>
#include <boost/math/distributions/poisson.hpp>
#include <boost/math/distributions/normal.hpp>
#include "rate_filter.hpp"
#include "controller.hh"
#include "timestamp.hh"
#include <unordered_set>

#define DEBUGGING if (debug_)

using namespace std;
using namespace boost::math;

Controller::Controller(const bool debug)
  : debug_(true)
{
    double dr = 1000.0 / 256;
    rates = new double[n_rates];
    probs = new double[n_rates];
    vector<double> v_rates;
    for (int i = 0; i < n_rates; i++)
    {
        rates[i] = (i + 1) * dr;
        probs[i] = 1.0 / 256;
        v_rates.push_back((i + 1) * dr);
    }

    rf = RateFilter(v_rates, n_rates, vol, n_thresh);
    recv_rf = RateFilter(v_rates, n_rates, vol, n_thresh);
}

unsigned int Controller::window_size()
{
    return cur_ws;
}

void Controller::datagram_was_sent(
		/* of the sent datagram */
        const uint64_t sequence_number,
        /* in milliseconds */
		const uint64_t send_timestamp)
{
    in_transit++;
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
    in_transit--;
    if (start_time == 0)
    {
        start_time = recv_timestamp_acked;
        last_received = timestamp_ack_received;
    }

    ts_t time_elapsed = recv_timestamp_acked - start_time;
    double secs = time_elapsed / 1000.0;
    double recv_secs = 0.001 * (timestamp_ack_received - last_received);
    num_acks++;
    if (time_elapsed > tick_length)
    {
        start_time = recv_timestamp_acked;
        last_received = timestamp_ack_received;
        rf.evolve(secs);
        rf.observe(num_acks, secs);

        double pred_rate = rf.get_predicted_rate();
        int expected_drain = quantile(poisson(pred_rate * 0.09), 0.05);
        int d_cur_ws = expected_drain - in_transit;
        cur_ws += ceil(0.6 * d_cur_ws);
        cur_ws = min(max(cur_ws, 3), 100);
        cout << "CW " << cur_ws << endl;
        link_rate = pred_rate;
        last_error = d_cur_ws;
        num_acks = 0;
    }
}

unsigned int Controller::timeout_ms()
{
    return 30;
}
