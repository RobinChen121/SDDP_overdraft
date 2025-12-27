/*
 * Created by Zhen Chen on 2025/12/27.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef SINGLE_PRODUCT_H
#define SINGLE_PRODUCT_H

#include "cash_leadtime_state.h"
#include "pmf.h"

class OverdraftLeadtimeSingleProduct {
    double ini_inventory = 0;
    double ini_cash = 0;
    CashLeadtimeState ini_state = CashLeadtimeState{1, ini_inventory, ini_cash, 0.0};
    std::vector<double> demands = {15.0, 15.0};
    size_t T = demands.size();

    std::vector<double> prices = std::vector<double>(T, 5.0);
    std::vector<double> unit_vari_order_costs = std::vector<double>(T, 1.0);
    std::vector<double> overhead_costs = std::vector<double>(T, 50.0);
    double unit_salvage_value = 0.5;

    double r0 = 0.0;
    double r1 = 0.05;
    double r2 = 2.0;
    double overdraft_limit = 300;

    double max_order_quantity = 60.0; // affect much
    double truncated_quantile = 0.9999;
    double step_size = 1.0;
    double min_inventory = 0;
    double max_inventory = 50;
    double min_cash = -200;
    double max_cash = 1000;

    std::vector<std::vector<std::array<double, 2>>> pmf;
    std::unordered_map<CashLeadtimeState, double> cache_actions;
    std::unordered_map<CashLeadtimeState, double> cache_values;

public:
    OverdraftLeadtimeSingleProduct() { pmf = get_pmf_poisson(demands, truncated_quantile); }

    OverdraftLeadtimeSingleProduct(const std::vector<double> &mean_demands, const double interest,
                                   const double overhead_cost, const double price) :
        demands(mean_demands), r1(interest) {
        overhead_costs = std::vector<double>(T, overhead_cost);
        prices = std::vector<double>(T, price);
        pmf = get_pmf_poisson(demands, truncated_quantile);
    }

    // [[nodiscard]] 表示：“函数的返回值不应该被忽略”
    // 如果你调用这个函数但没有使用返回值，编译器会给你一个警告，提醒你可能写错了。
    [[nodiscard]] std::vector<double> feasibleActions() const;
    [[nodiscard]] double immediateValueFunction(const CashLeadtimeState &state, const double action,
                                                const double randomDemand) const;
    [[nodiscard]] CashLeadtimeState stateTransitionFunction(const CashLeadtimeState &state,
                                                            const double action,
                                                            const double randomDemand) const;
    double recursion(const CashLeadtimeState &state);
    std::vector<double> solve();
};

#endif // SINGLE_PRODUCT_H
