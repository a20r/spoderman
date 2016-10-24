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

void Controller::evolve(double time)
{
    // _normalized = false;

    /* initialize brownian motion */
    double std = vol * sqrt(time);
    normal brownian(0, std);
    // _gaussian.calculate( stddev );


    /* initialize new pmf */
    // SampledFunction new_pmf( _probability_mass_function );
    // new_pmf.for_each( [] ( const double, double & value, const unsigned int ) { value = 0; } );

    double new_probs[n_rates];
    double new_rates[n_rates];
    copy(rates, rates + n_rates, new_rates);
    copy(probs, probs + n_rates, new_probs);
    // fill_n(new_probs, n_rates, 0.0);

    // const double zero_escape_probability = 1 - poissonpdf( time * _outage_escape_rate, 0 );
    //
    for (int i = 0; i < n_rates; i++)
    {
        // cout << new_probs[i] << endl;
    }

    for (int i = 0; i < n_rates; i++)
    {
        double old_rate = rates[i];
        double old_prob = probs[i];

        for (int j = 0; j < n_rates; j++)
        {
            bool gt = new_rates[j] >= old_rate - 5 * std;
            bool lt = new_rates[j] <= old_rate + 5 * std;
            if (gt and lt)
            {
                double bottom = min_rate * (j + 1);
                double top = min_rate * (j + 2);
                double bottom_cdf = cdf(brownian, bottom - old_rate);
                double top_cdf = cdf(brownian, top - old_rate);
                double cont = old_prob * (top_cdf - bottom_cdf);
                new_probs[j] += cont;
            }
        }
    }

    // double f_sum = 0.0;
    // for (int i = 0; i < n_rates; i++)
    // {
    //     // cout << "NEW_PROB " << new_probs[i] << endl;
    //     f_sum += new_probs[i];
    // }
    //
    // cout << "FSUM " << f_sum << endl;
    //
    // for (int i = 0; i < n_rates; i++)
    // {
    //     new_probs[i] = new_probs[i] / f_sum;
    // }

    // memcpy(probs, new_probs, n_rates);
    // memcpy(rates, new_rates, n_rates);
    copy(new_rates, new_rates + n_rates, rates);
    copy(new_probs, new_probs + n_rates, probs);



  // _probability_mass_function.for_each( [&]
  //                      ( const double old_rate, const double & old_prob, const unsigned int old_index )
  //                      {
  //                    new_pmf.for_range( old_rate - 5 * stddev,
  //                               old_rate + 5 * stddev,
  //                               [&]
  //                               ( const double new_rate, double & new_prob, const unsigned int new_index )
  //                               {
  //                                 double zfactor = 1.0;
  //
  //                                 if ( old_index == 0 ) {
  //                               zfactor = ( new_index != 0 ) ? zero_escape_probability : (1 - zero_escape_probability);
  //                                 }
  //
  //                                 double contribution = zfactor * old_prob
  //                               * ( _gaussian.cdf( new_pmf.sample_ceil( new_rate ) - old_rate )
  //                                   - _gaussian.cdf( new_pmf.sample_floor( new_rate ) - old_rate ) );
  //
  //                                 new_prob += contribution;
  //                               } );
  //                      } );
  //
  // _probability_mass_function = new_pmf;
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
    }

    ts_t time_elapsed = recv_timestamp_acked - start_time;
    double secs = time_elapsed / 1000.0;
    num_acks++;
    if (time_elapsed > tick_length)
    {
        // cout << "TIME: " << time_elapsed << endl;
        link_rate = 1000 * num_acks / (double) time_elapsed;
        start_time = recv_timestamp_acked;

        double fs[n_rates];
        double f_sum = 0.0;
        // evolve_rates(rates, secs);
        evolve(secs);
        for (int i = 0; i < n_rates; i++)
        {
            double p = pdf(poisson(rates[i] * secs), num_acks);
            // cout << probs[i] << " " << p << " " << endl;
            fs[i] = probs[i] * p;
            f_sum += fs[i];
        }

        // cout << "SUM " << f_sum << endl;
        double pred_rate = 0.0;
        for (int i = 0; i < n_rates; i++)
        {
            probs[i] = fs[i] / f_sum;
            pred_rate += rates[i] * probs[i];
        }

        // double forecast_rates[n_rates];
        // memcpy(forecast_rates, rates, n_rates);
        //
        // int new_ws = 0;
        // int last_ws = 0;
        // int queue_estimate = in_transit;
        // for (int tick = 0; tick < 8; tick++)
        // {
        //     evolve_rates(forecast_rates, tick_length * 0.001);
        //     double pred_rate = 0.0;
        //     for (int i = 0; i < n_rates; i++)
        //     {
        //         pred_rate += forecast_rates[i] * probs[i];
        //     }
        //
        //     double adjusted_rate = pred_rate * 100 * 0.001;
        //     int expected_drain = quantile(poisson(adjusted_rate), 0.05);
        //     queue_estimate = max(queue_estimate - expected_drain, 0);
        //     int to_send = max(queue_estimate - expected_drain, 1);
        //     queue_estimate += (to_send - expected_drain);
        //     queue_estimate = max(queue_estimate, 1);
        //
        // }

        cout << "RATE " << pred_rate << endl;
        cur_ws = max(quantile(poisson(pred_rate * 0.1), 0.05), 10.0);
        cout << "CW " << cur_ws << endl;
        num_acks = 0;
    }
}

unsigned int Controller::timeout_ms()
{
    return 1000;
}
