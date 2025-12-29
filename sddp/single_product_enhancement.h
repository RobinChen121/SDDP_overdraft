/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description: 
 *
 *
 */

#ifndef SINGLE_PRODUCT_ENHANCEMENT_H
#define SINGLE_PRODUCT_ENHANCEMENT_H

#include <vector>

class SingleProduct {
    // problem settings
    double iniI = 0;
    double iniCash = 0;
    std::vector<double> mean_demands = {15.0, 15.0, 15.0, 15.0}; // std::vector<double>(4, 15);
    std::string distribution_name = "poisson";
    size_t T = mean_demands.size();
    std::vector<double> unit_vari_costs = std::vector<double>(T, 1);
    std::vector<double> prices = std::vector<double>(T, 10);
    double unit_salvage_value = 0.5;
    std::vector<double> overhead_costs = std::vector<double>(T, 50);
    double r0 = 0; // interest rate
    double r1 = 0.1;
    double r2 = 2;
    double overdraft_limit = 500;

    // sddp settings
    int sample_num = 10;
    int forward_num = 30;
    int iter_num = 50;
    double theta_initial_value = -500;

public:
    SingleProduct() {};
    SingleProduct(const std::vector<double> &mean_demands, const double price, const double r1,
                  const double overhead_cost, const int sample_num, const int forward_num,
                  const int iter_num)
        : mean_demands(mean_demands), r1(r1), sample_num(sample_num), forward_num(forward_num),
          iter_num(iter_num) {
        prices = std::vector(T, price);
        overhead_costs = std::vector(T, overhead_cost);
    };

    [[nodiscard]] std::array<double, 2> solve() const;
};

#endif //SINGLE_PRODUCT_ENHANCEMENT_H
