#include <iostream>
#include <cmath>
#include <boost/math/distributions/poisson.hpp>
#include <boost/math/distributions/normal.hpp>
#include "controller.hh"
#include "timestamp.hh"

#define DEBUGGING if (debug_)

using namespace std;
using namespace boost::math;

Controller::Controller(const bool debug)
  : debug_(true)
{
    double dr = 1000.0 / 256;
    rates = new double[n_rates];
    probs = new double[n_rates];
    for (int i = 0; i < n_rates; i++)
    {
        rates[i] = (i + 1) * dr;
        probs[i] = 1.0 / 256;
    }
}

unsigned int Controller::window_size()
{
    // if (in_transit > 50)
    // {
    //     return 10;
    // }
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

long factorial(long x)
{
    long prod = 1;
    for (long i = 1; i <= x; i++)
    {
        prod *= i;
    }

    return prod;
}

void Controller::evolve_rates(double *rs, double secs)
{
    std::normal_distribution<double> dist(0, sqrt(secs) * vol);
    double noise = dist(gen);
    for (int i = 0; i < n_rates; i++)
    {
        rs[i] += noise;
        if (rs[i] < min_rate)
        {
            rs[i] = min_rate;
        }
    }
}

void Controller::evolve(double time, double *rs, double *ps)
{

    double std = vol * sqrt(time);
    normal brownian(0, std);


    double new_probs[n_rates];
    double new_rates[n_rates];
    copy(rs, rs + n_rates, new_rates);
    // copy(ps, ps + n_rates, new_probs);
    fill_n(new_probs, n_rates, 0);

    for (int i = 0; i < n_rates; i++)
    {
        double old_rate = rs[i];
        double old_prob = ps[i];

        for (int j = 0; j < n_rates; j++)
        {
            bool gt = new_rates[j] >= old_rate - 3 * std;
            bool lt = new_rates[j] <= old_rate + 3 * std;
            if (gt and lt)
            {
                double bottom = min_rate * (j + 1) - min_rate / 2;
                double top = min_rate * (j + 1) + min_rate / 2;
                double bottom_cdf = cdf(brownian, bottom - old_rate);
                double top_cdf = cdf(brownian, top - old_rate);
                double cont = old_prob * (top_cdf - bottom_cdf);
                new_probs[j] += cont;
            }
        }
    }

    copy(new_rates, new_rates + n_rates, rs);
    copy(new_probs, new_probs + n_rates, ps);
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
        send_time = send_timestamp_acked;
    }

    ts_t time_elapsed = recv_timestamp_acked - start_time;
    ts_t send_time_elapsed = send_timestamp_acked - send_time;
    double secs = time_elapsed / 1000.0;
    num_acks++;
    if (time_elapsed > tick_length)
    {
        // cout << "TIME: " << time_elapsed << endl;
        double send_rate = 1000 * num_acks / (double) send_time_elapsed;
        double escape_rate = 1000 * num_acks / (double) time_elapsed;
        // cout << send_rate - escape_rate << endl;
        start_time = recv_timestamp_acked;
        send_time = send_timestamp_acked;

        double fs[n_rates];
        double f_sum = 0.0;
        // evolve_rates(rates, secs);
        evolve(secs, rates, probs);
        for (int i = 0; i < n_rates; i++)
        {
            double p = pdf(poisson(rates[i] * secs), num_acks);
            probs[i] = probs[i] * p;
        }

        // evolve(0.1, rates, probs);
        for (int i = 0; i < n_rates; i++)
        {
            f_sum += probs[i];
        }

        double pred_rate = 0.0;
        for (int i = 0; i < n_rates; i++)
        {
            probs[i] = probs[i] / f_sum;
            pred_rate += rates[i] * probs[i];
        }

        double forecast_rates[n_rates];
        double forecast_probs[n_rates];
        copy(rates, rates + n_rates, forecast_rates);
        copy(probs, probs + n_rates, forecast_probs);

        int new_ws = 0;
        int last_ws = 0;
        int queue_estimate = in_transit;
        // for (int tick = 0; tick < 8; tick++)
        // {
        //     evolve(tick_length * 0.001, forecast_rates, forecast_probs);
        //
        //     double f_sum = 0.0;
        //     for (int i = 0; i < n_rates; i++)
        //     {
        //         f_sum += forecast_probs[i];
        //     }
        //
        //     double pred_rate = 0.0;
        //     for (int i = 0; i < n_rates; i++)
        //     {
        //         forecast_probs[i] /= f_sum;
        //         pred_rate += forecast_rates[i] * forecast_probs[i];
        //     }
        //
        //     double adjusted_rate = pred_rate * 0.1;
        //     int expected_drain = quantile(poisson(adjusted_rate), 0.05);
        //     int d_cur_ws = expected_drain - queue_estimate;
        //     int safe_to_send = ceil(0.7 * d_cur_ws);
        //     new_ws += max(safe_to_send - last_ws, 0);
        //     last_ws = new_ws;
        //     queue_estimate = max(queue_estimate + safe_to_send - expected_drain, 0);
        // }

        // cur_ws = max(new_ws, 3);

        // cur_ws = max(new_ws, 10);
        // cur_ws = 2;
        // cout << "RATE " << pred_rate << endl;

        int expected_drain = quantile(poisson(pred_rate * 0.1), 0.05);
        // cout << expected_drain << " " << in_transit << endl;
        int d_cur_ws = expected_drain - in_transit;
        // if (d_cur_ws > 0)
        // {
        //     cur_ws += ceil(0.7 * d_cur_ws);
        // }
        // else
        // {
        //     cur_ws += d_cur_ws;
        // }
        cur_ws += ceil(0.7 * d_cur_ws);
        cur_ws = min(max(cur_ws, 3), 70);
        cout << "CW " << cur_ws << endl;
        num_acks = 0;
    }
}

unsigned int Controller::timeout_ms()
{
    return 30;
}
