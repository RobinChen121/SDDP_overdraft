/*
 * Created by Zhen Chen on 2025/12/30.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef DOUBLE_PRODUCT_H
#define DOUBLE_PRODUCT_H

#include <vector>

class DoubleProduct {
    // problem settings
    double ini_I = 0;
    double ini_cash = 0;

    std::vector<double> mean_demand1 = {30, 30, 30};
    size_t T = mean_demand1.size(); // 直接获取大小
    std::vector<double> mean_demand2 = std::vector<double>(T);
//    std::vector<double> demand1_weights = std::vector{0.25, 0.5, 0.25};
//    std::vector<double> demand2_weights = std::vector<double>(T);

    std::vector<double> prices1 = std::vector<double>(T, 5.0);
    std::vector<double> prices2 = std::vector<double>(T, 10.0);
    std::vector<double> unit_vari_order_costs1 = std::vector<double>(T, 1.0);
    std::vector<double> unit_vari_order_costs2 = std::vector<double>(T, 2.0);
    std::vector<double> overhead_costs = std::vector<double>(T, 100.0);
    double unit_salvage_value1 = 0.5 * unit_vari_order_costs1[T - 1];
    double unit_salvage_value2 = 0.5 * unit_vari_order_costs2[T - 1];

    double r0 = 0.0;
    double r1 = 0.1;
    double r2 = 2.0;
    double overdraft_limit = 500;

    // sddp settings
    int sample_num = 10; // 10;
    int forward_num = 20; // 20;
    int iter_num = 50;
    double theta_initial_value = -1000;

public:
    DoubleProduct() {
        std::ranges::transform(mean_demand1, mean_demand2.begin(),
                               [](const double x) { return x / 2; });
    }

    DoubleProduct(const std::vector<double> &mean_demands, const double interest, double limit,
                  const int sample_num, const int forward_num, const int iter_num) :
        mean_demand1(mean_demands), r1(interest), overdraft_limit(limit), sample_num(sample_num),
        forward_num(forward_num), iter_num(iter_num) {
        std::ranges::transform(mean_demand1, mean_demand2.begin(),
                               [](const double x) { return x / 2; });
    }

    std::array<double, 3> solve() const;
};

#endif // DOUBLE_PRODUCT_H
