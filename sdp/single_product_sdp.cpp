/*
 * Created by Zhen Chen on 2025/12/27.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "single_product_sdp.h"
#include <iostream>
#include "cash_leadtime_state.h"

[[nodiscard]] std::vector<double> OverdraftLeadtimeSingleProduct::get_feasible_actions() const {
    const int q_num = static_cast<int>(max_order_quantity);

    std::vector<double> actions(q_num);
    for (int i = 0; i < q_num; i = i + 1) {
        actions[i] = i;
    }
    return actions;
}

[[nodiscard]] double OverdraftLeadtimeSingleProduct::immediate_value_function(
        const CashLeadtimeState &state, const double action, const double random_demand) const {
    const int t = state.get_period() - 1;
    const double revenue =
            prices[t] * std::min(state.get_ini_inventory() + state.get_q_pre(), random_demand);
    const double variable_cost = unit_vari_order_costs[t] * action;
    const double inventory = state.get_ini_inventory() + state.get_q_pre() - random_demand;
    const double cash_before_interest = state.get_ini_cash() - variable_cost - overhead_costs[t];
    double interest = 0;
    if (cash_before_interest > 0.0) {
        interest = cash_before_interest * r0;
    } else if (-overdraft_limit < cash_before_interest && cash_before_interest < 0.0) {
        interest = cash_before_interest * r1;
    } else {
        interest = -overdraft_limit * r1 + (cash_before_interest - overdraft_limit) * r2;
        // interest = cash_before_interest * r2;
    }
    const double cash_after_interest = cash_before_interest + interest + revenue;
    double cash_increment = cash_after_interest - state.get_ini_cash();
    const double salvage_value =
            state.get_period() == T ? unit_salvage_value * std::max(inventory, 0.0) : 0;
    cash_increment += salvage_value;
    return cash_increment;
}

[[nodiscard]] CashLeadtimeState OverdraftLeadtimeSingleProduct::state_transition_function(
        const CashLeadtimeState &state, const double action, const double random_demand) const {
    double next_inventory =
            std::max(0.0, state.get_ini_inventory() + state.get_q_pre() - random_demand);
    const double nextQpre = action;
    double nextCash = state.get_ini_cash() + immediate_value_function(state, action, random_demand);
    nextCash = nextCash > max_cash ? max_cash : nextCash;
    nextCash = nextCash < min_cash ? min_cash : nextCash;
    next_inventory = next_inventory > max_inventory ? max_inventory : next_inventory;
    next_inventory = next_inventory < min_inventory ? min_inventory : next_inventory;
    return CashLeadtimeState{state.get_period() + 1, next_inventory, nextCash, nextQpre};
}

double OverdraftLeadtimeSingleProduct::recursion(const CashLeadtimeState &state) { // NOLINT
    double best_q = 0.0;
    double best_value = std::numeric_limits<double>::lowest();
    const std::vector<double> actions = get_feasible_actions();
    for (const double action: actions) {
        double this_value = 0.0;
        for (auto demand_and_prob: pmf[state.get_period() - 1]) {
            this_value += demand_and_prob[1] *
                          immediate_value_function(state, action, demand_and_prob[0]);
            if (state.get_period() < T) {
                auto new_state = state_transition_function(state, action, demand_and_prob[0]);
                auto it = cache_values.find(new_state);
                if (it != cache_values.end()) {
                    this_value += demand_and_prob[1] * it->second;
                } else {
                    this_value += demand_and_prob[1] * recursion(new_state);
                }
            }
        }
        if (this_value > best_value) {
            best_value = this_value;
            best_q = action;
        }
    }
    cache_actions[state] = best_q;
    cache_values[state] = best_value;
    return best_value;
}

std::vector<double> OverdraftLeadtimeSingleProduct::solve() {
    return {recursion(ini_state), cache_actions.at(ini_state)};
}

int main() {
    auto problem = OverdraftLeadtimeSingleProduct();
    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto final_value = problem.solve();
    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> time = end_time - start_time;
    std::cout << "running time is " << time.count() << 's' << std::endl;
    std::cout << "Final expected cash increment is " << final_value[0] << std::endl;
    std::cout << "Optimal Q in the first period is " << final_value[1] << std::endl;

    return 0;
}
