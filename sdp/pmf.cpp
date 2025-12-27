/*
 * Created by Zhen Chen on 2025/12/27.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "pmf.h"


double poisson_pmf(const int k, const int lambda) {
    if (k < 0 || lambda < 0)
        return 0.0;
    if (k == 0 and lambda == 0)
        return 1.0;

    const double logP = -lambda + k * std::log(lambda) - std::lgamma(k + 1);
    return std::exp(logP); // Use the logarithmic form to avoid overflow from std::tgamma(k + 1)
}

double poisson_cdf(const int k, const double lambda) {
    double cumulative = 0.0;
    double term = std::exp(-lambda);
    for (int i = 0; i <= k; ++i) {
        cumulative += term;
        if (i < k)
            term *= lambda / (i + 1);
    }

    return cumulative;
}

int poisson_quantile(const double p, const double lambda) {
    int low = 0, high = std::max(100, static_cast<int>(lambda * 3));
    while (low < high) {
        if (const int mid = (low + high) / 2; poisson_cdf(mid, lambda) < p) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return low;
}

std::vector<std::vector<std::array<double, 2>>> get_pmf_poisson(const std::vector<double> &demands,
                                                                const double quantile) {
    const size_t T = demands.size();
    std::vector<int> support_lb(T);
    std::vector<int> support_ub(T);
    for (size_t i = 0; i < T; ++i) {
        support_ub[i] = poisson_quantile(quantile, demands[i]);
        support_lb[i] = poisson_quantile(1 - quantile, demands[i]);
    }
    std::vector pmf(T, std::vector<std::array<double, 2>>());
    for (int t = 0; t < T; ++t) {
        const int demand_length = support_ub[t] - support_lb[t] + 1;
        pmf[t].resize(demand_length, std::array<double, 2>());
        for (int j = 0; j < demand_length; ++j) {
            pmf[t][j][0] = support_lb[t] + j;
            const int demand = static_cast<int>(pmf[t][j][0]);
            pmf[t][j][1] = poisson_pmf(demand, static_cast<int>(demands[t])) / (2 * quantile - 1);
        }
    }
    return pmf;
}
