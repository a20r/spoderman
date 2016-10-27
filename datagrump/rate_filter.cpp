
#include <iostream>
#include <cmath>
#include <boost/math/distributions/poisson.hpp>
#include <boost/math/distributions/normal.hpp>
#include <vector>
#include "rate_filter.hpp"

using namespace std;
using namespace boost::math;

RateFilter::RateFilter()
{
}

RateFilter::RateFilter(vector<double>& rates, int n_rates, double vol,
        double resample_thresh) :
    rates(rates), n_rates(n_rates), vol(vol), resample_thresh(resample_thresh)
{

    this->probs = vector<double>(n_rates, 1.0 / n_rates);
}

RateFilter::RateFilter(vector<double>& rates, vector<double> &probs,
        int n_rates, double vol,
        double resample_thresh) :
    rates(rates), probs(probs), n_rates(n_rates), vol(vol),
    resample_thresh(resample_thresh)
{
}

RateFilter::~RateFilter()
{
}

void RateFilter::observe(int num_acks, double secs)
{
    resample();

    for (int i = 0; i < n_rates; i++)
    {
        double p = pdf(poisson(rates[i] * secs), num_acks);
        probs[i] = probs[i] * p;
    }

    normalize();
}

void RateFilter::normalize()
{
    double total = 0.0;
    for (int i = 0; i < n_rates; i++)
    {
        total += probs[i];
    }

    for (int i = 0; i < n_rates; i++)
    {
        probs[i] /= total;
    }
}

void RateFilter::evolve(double secs)
{
    double std = vol * sqrt(secs);
    normal brownian(0, std);

    double new_probs[n_rates];
    fill_n(new_probs, n_rates, 0);

    for (int i = 0; i < n_rates; i++)
    {
        double old_prob = probs[i];

        for (int j = 0; j < n_rates; j++)
        {
            bool gt = rates[j] >= rates[i] - 2 * std;
            bool lt = rates[j] <= rates[i] + 2 * std;
            if (gt and lt)
            {
                double bottom = rates[j] - rates[0] / 2;
                double top = rates[j] + rates[0] / 2;
                double bottom_cdf = cdf(brownian, bottom - rates[i]);
                double top_cdf = cdf(brownian, top - rates[i]);
                double cont = old_prob * (top_cdf - bottom_cdf);
                new_probs[j] += cont;
            }
        }
    }

    copy(new_probs, new_probs + n_rates, probs.begin());
    normalize();
}

void RateFilter::resample()
{
    double n_eff_inv = 0.0, n_eff;
    for (int i = 0; i < n_rates; i++)
    {
        n_eff_inv += probs[i] * probs[i];
    }

    n_eff = 1.0 / n_eff_inv;
    // cout << n_eff << endl;

    if (n_eff < resample_thresh)
    {
        for (int i = 0; i < n_rates; i++)
        {
            // probs[i] += resample_weight / n_rates;
            probs[i] = 1.0 / 256;
        }
        cout << "============== RESAMPLING ==============" << endl;
    }
}

double RateFilter::get_probability(int i)
{
    return probs[i];
}

double RateFilter::get_rate(int i)
{
    return rates[i];
}

double RateFilter::get_predicted_rate()
{
    double pred_rate = 0.0;
    for (int i = 0; i < n_rates; i++)
    {
        pred_rate += rates[i] * probs[i];
    }
    return pred_rate;
}

RateFilter RateFilter::clone()
{
    return RateFilter(rates, probs, n_rates, vol, resample_thresh);
}
