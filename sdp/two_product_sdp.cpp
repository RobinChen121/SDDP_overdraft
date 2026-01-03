/*
 * Created by Zhen Chen on 2025/12/28.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */
#include "cash_leadtime_state_multi.h"
#include "pmf.h"

#include <algorithm>
#include <iostream>
#include <vector>

class OverdraftLeadtimeDoubleProduct {
private:
  double ini_inventory1 = 0.0;
  double ini_inventory2 = 0.0;
  double ini_cash = 0.0;
  double ini_q_pre1 = 0.0;
  double ini_q_pre2 = 0.0;

  CashLeadtimeMultiState ini_state{1,         ini_inventory1, ini_inventory2,
                                   ini_q_pre1, ini_q_pre2,      ini_cash};

  std::vector<double> mean_demand1 = {15, 15, 15};
  size_t T = 2; // mean_demand1.size(); // 直接获取大小
  std::vector<double> mean_demand2 = std::vector<double>(T);
  std::string distribution_type = "poisson";

  std::vector<double> prices1 = std::vector<double>(T, 5.0);
  std::vector<double> prices2 = std::vector<double>(T, 10.0);
  std::vector<double> unit_vari_order_costs1 = std::vector<double>(T, 1.0);
  std::vector<double> unit_vari_order_costs2 = std::vector<double>(T, 2.0);
  std::vector<double> overhead_costs = std::vector<double>(T, 50.0);
  double unit_salvage_value1 = 0.5 * unit_vari_order_costs1[T - 1];
  double unit_salvage_value2 = 0.5 * unit_vari_order_costs2[T - 1];

  double r0 = 0.0;
  double r1 = 0.1;
  double r2 = 2.0;
  double overdraft_limit = 500;

  double max_order_quantity1 = 25.0;
  double max_order_quantity2 = 15.0;
  double truncated_quantile = 0.9999;
  double step_size = 1.0;
  double min_inventory = 0;
  double max_inventory = 50;
  double min_cash = -200;
  double max_cash = 300;

  std::vector<std::vector<std::array<double, 3>>> pmf;
  std::unordered_map<CashLeadtimeMultiState, std::array<double, 2>> cache_actions;
  std::unordered_map<CashLeadtimeMultiState, double> cache_values;

public:
  OverdraftLeadtimeDoubleProduct() {

    std::ranges::transform(mean_demand1, mean_demand2.begin(),
                           [](const double x) { return x / 2; });
    pmf =
        get_pmf_poisson_multi(mean_demand1, mean_demand2, truncated_quantile);

  }

  [[nodiscard]] std::vector<std::array<double, 2>> get_feasible_actions() const {
    const int q_num1 = static_cast<int>(max_order_quantity1 / step_size);
    const int q_num2 = static_cast<int>(max_order_quantity2 / step_size);
    const int q_total_num = q_num1 * q_num2;
    int index = 0;
    std::vector<std::array<double, 2>> actions(q_total_num);
    for (int i = 0; i < q_num1; i = i + 1) {
      const double action1 = i * step_size;
      for (int j = 0; j < q_num2; j = j + 1) {
        const double action2 = j * step_size;
        actions[index] = {action1, action2};
        index += 1;
      }
    }
    return actions;
  }

  [[nodiscard]] double immediate_value_function(const CashLeadtimeMultiState &state,
                                                const double action1, const double action2,
                                                const double random_demand1,
                                                const double random_demand2) const {
    const int t = state.get_period() - 1;
    const double revenue =
        prices1[t] * std::min(state.get_iniI1() + state.get_q_pre1(), random_demand1) +
        prices2[t] * std::min(state.get_iniI2() + state.get_q_pre2(), random_demand2);
    const double variableCost =
        unit_vari_order_costs1[t] * action1 + unit_vari_order_costs2[t] * action2;
    const double inventory1 = state.get_iniI1() + state.get_q_pre1() - random_demand1;
    const double inventory2 = state.get_iniI2() + state.get_q_pre2() - random_demand2;
    const double cash_before_interest = state.get_ini_cash() - variableCost - overhead_costs[t];
    double interest = 0;
    if (cash_before_interest > 0.0) {
      interest = cash_before_interest * r0;
    } else if (-overdraft_limit < cash_before_interest && cash_before_interest < 1e-6) {
      interest = cash_before_interest * r1;
    } else {
      interest = -overdraft_limit * r1 + (cash_before_interest - overdraft_limit) * r2;
    }
    const double cash_after_interest = cash_before_interest + interest + revenue;
    double cash_increment = cash_after_interest - state.get_ini_cash();
    const double salvage_value = state.get_period() == T
                                ? unit_salvage_value1 * std::max(inventory1, 0.0) +
                                      unit_salvage_value2 * std::max(inventory2, 0.0)
                                : 0;
    cash_increment += salvage_value;
    return cash_increment;
  }

  [[nodiscard]] CashLeadtimeMultiState
  state_transition_function(const CashLeadtimeMultiState &state, const double action1,
                            const double action2, const double random_demand1,
                            const double random_demand2) const {
    double next_inventory1 = std::max(0.0, state.get_iniI1() + state.get_q_pre1() - random_demand1);
    double next_inventory2 = std::max(0.0, state.get_iniI2() + state.get_q_pre2() - random_demand2);
    const double next_q_pre1 = action1;
    const double next_q_pre2 = action2;
    double next_cash = state.get_ini_cash() + immediate_value_function(state, action1, action2,
                                                                      random_demand1, random_demand2);
    next_cash = next_cash > max_cash ? max_cash : next_cash;
    next_cash = next_cash < min_cash ? min_cash : next_cash;
    next_inventory1 = next_inventory1 > max_inventory ? max_inventory : next_inventory1;
    next_inventory1 = next_inventory1 < min_inventory ? min_inventory : next_inventory1;
    next_inventory2 = next_inventory2 > max_inventory ? max_inventory : next_inventory2;
    next_inventory2 = next_inventory2 < min_inventory ? min_inventory : next_inventory2;
//    // cash is integer or not
//    next_cash = std::round(next_cash * 10) / 10.0; // the right should be a
//    // decimal
    return CashLeadtimeMultiState{
        state.get_period() + 1, next_inventory1, next_inventory2, next_q_pre1, next_q_pre2, next_cash};
  }

  double recursion(const CashLeadtimeMultiState &state) { // NOLINT(*-no-recursion)
    std::array best_q = {0.0, 0.0};
    double best_value = std::numeric_limits<double>::lowest();
    const std::vector<std::array<double, 2>> actions =
        get_feasible_actions(); // should not put in the loop
    for (const std::array action : actions) {
      double this_value = 0.0;
      for (auto demand_and_prob : pmf[state.get_period() - 1]) {
        this_value +=
            demand_and_prob[2] * immediate_value_function(state, action[0], action[1],
                                                        demand_and_prob[0], demand_and_prob[1]);
        if (state.get_period() < T) {
          auto new_state = state_transition_function(state, action[0], action[1], demand_and_prob[0],
                                                    demand_and_prob[1]);
          if (auto it = cache_values.find(new_state); it != cache_values.end()) {
            // some issues here
            this_value += demand_and_prob[2] * it->second;
          } else {
            this_value += demand_and_prob[2] * recursion(new_state);
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

  std::vector<double> solve() {
    return {recursion(ini_state), cache_actions.at(ini_state)[0], cache_actions.at(ini_state)[1]};
  }
};


int main() {
  auto problem = OverdraftLeadtimeDoubleProduct();
  const auto start_time = std::chrono::high_resolution_clock::now();
  const auto final_value = problem.solve()[0];
  const auto end_time = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> time = end_time - start_time;
  std::cout << "Final expected cash increment is " << final_value << std::endl;
  std::cout << "Optimal Qs in the first period is " << problem.solve()[1] << " and "
            << problem.solve()[2] << std::endl;
  std::cout << "running time is " << time.count() << std::endl;

  return 0;
}