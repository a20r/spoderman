
#ifndef RATE_FILTER_HPP
#define RATE_FILTER_HPP

using namespace std;

#include <vector>

class RateFilter
{
    private:
        vector<double> rates;
        vector<double> probs;
        double vol;
        double resample_thresh;
        int n_rates = 0;

    public:
        RateFilter();
        RateFilter(vector<double>& rates, int n_rates, double vol,
                double resample_thresh);
        RateFilter(vector<double>& rates, vector<double>& probs,
                int n_rates, double vol,
                double resample_thresh);
        ~RateFilter();
        void observe(int num_acks, double secs);
        double get_probability(int i);
        double get_rate(int i);
        void normalize();
        double get_predicted_rate();
        void resample();
        void evolve(double secs);
        RateFilter clone();
};

#endif
