/*
 * Created by Zhen Chen on 2025/12/27.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef PMF_H
#define PMF_H

#include <vector>

double poisson_pmf(int k, int lambda);
double poisson_cdf(int k, double lambda);
int poisson_quantile(double p, double lambda);

std::vector<std::vector<std::array<double, 2>>> get_pmf_poisson(const std::vector<double> &demands,
                                                                double quantile);

#endif // PMF_H
