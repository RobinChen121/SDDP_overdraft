/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "single_product.h"
#include <numeric>
#include "sampling.h"

std::array<double, 2> SingleProduct::solve() const {
    const std::vector sample_nums(T, sample_num);
    std::vector<std::vector<double>> sample_details(T);
    for (int t = 0; t < T; t++) {
        sample_details[t].resize(sample_nums[t]);
        sample_details[t] = generate_samples_poisson(sample_nums[t], mean_demands[t]);
    }

    // gurobi environments and model
    auto env = GRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    auto model = GRBModel(env);
    std::vector models(T + 1, GRBModel(env));

    // decision variables
    std::vector<GRBVar> q(T);
    std::vector<GRBVar> q_pre(T - 1);
    std::vector<GRBVar> theta(T); // previous is T - 1, which causing the error
    std::vector<GRBVar> I(T);
    std::vector<GRBVar> B(T);
    std::vector<GRBVar> cash(T);
    std::vector<GRBVar> W0(T);
    std::vector<GRBVar> W1(T);
    std::vector<GRBVar> W2(T);

    // build initial models for each stage
    for (int t = 0; t < T + 1; t++) {
        if (t > 0) {
            cash[t - 1] = models[t].addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                           "cash_" + std::to_string(t));
            I[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "I_" + std::to_string(t));
            B[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "B_" + std::to_string(t));
            models[t].addConstr(I[t - 1] - B[t - 1] == 0);
            if (t < T) {
                models[t].addConstr(cash[t - 1] + prices[t - 1] * B[t - 1] == 0);
                q_pre[t - 1] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                                "q_pre_" + std::to_string(t + 1));
                models[t].addConstr(q_pre[t - 1] == 0);
            }
        }
        if (t < T) {
            q[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                    "q_" + std::to_string(t + 1));
            W0[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W0_" + std::to_string(t + 1));
            W1[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W1_" + std::to_string(t + 1));
            W2[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W2_" + std::to_string(t + 1));
            theta[t] = models[t].addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                        "theta_" + std::to_string(t + 2));
            models[t].addConstr(W1[t] <= overdraft_limit);
            if (t == 0)
                models[t].addConstr(ini_cash - unit_vari_costs[t] * q[t] - W0[t] + W1[t] + W2[t] ==
                                    overhead_costs[t]);
            else {
                models[t].addConstr(cash[t - 1] - unit_vari_costs[t] * q[t] - W0[t] + W1[t] +
                                            W2[t] ==
                                    overhead_costs[t]);
            }
            models[t].addConstr(theta[t] >= theta_initial_value * (static_cast<double>(T) - t));
        }
        if (t == 0)
            models[t].setObjective(overhead_costs[0] + unit_vari_costs[0] * q[0] + r2 * W2[0] +
                                   r1 * W1[0] - r0 * W0[0] + theta[0]);
        models[t].update();
    }

    double intercepts[iter_num][T][forward_num];
    double slopes3[iter_num][T][forward_num];
    double slopes2[iter_num][T][forward_num];
    double slopes1[iter_num][T][forward_num];
    double q_pre_values[iter_num][T][forward_num];
    double q_values[iter_num][T][forward_num];

    double I_forward_values[iter_num][T][forward_num];
    double W0_forward_values[iter_num][T][forward_num];
    double W1_forward_values[iter_num][T][forward_num];
    double W2_forward_values[iter_num][T][forward_num];
    int iter = 0;
    while (iter < iter_num) {
        auto scenarioPaths = generate_scenario_paths(forward_num, sample_nums);

        if (iter > 0) {
            models[0].addConstr(
                    theta[0] >=
                    slopes1[iter - 1][0][0] * ini_I +
                            slopes2[iter - 1][0][0] *
                                    ((1 + r0) * W0[0] - (1 + r1) * W1[0] - (1 + r2) * W2[0]) +
                            slopes3[iter - 1][0][0] * q[0] + intercepts[iter - 1][0][0]);
            models[0].update();
        }

        models[0].optimize();

        for (int n = 0; n < forward_num; n++) {
            q_values[iter][0][n] = q[0].get(GRB_DoubleAttr_X);
            W0_forward_values[iter][0][n] = W0[0].get(GRB_DoubleAttr_X);
            W1_forward_values[iter][0][n] = W1[0].get(GRB_DoubleAttr_X);
            W2_forward_values[iter][0][n] = W2[0].get(GRB_DoubleAttr_X);
        }

        // forward
        for (int t = 1; t < T + 1; t++) {
            if (iter > 0 && t < T) {
                std::vector<std::vector<double>> cut_coefficients(forward_num,
                                                                  std::vector<double>(4, 0));
                for (int n = 0; n < forward_num; n++) {
                    cut_coefficients[n][0] = slopes1[iter - 1][t][n];
                    cut_coefficients[n][1] = slopes2[iter - 1][t][n];
                    cut_coefficients[n][2] = slopes3[iter - 1][t][n];
                    cut_coefficients[n][3] = intercepts[iter - 1][t][n];
                };

                for (auto &coefficient: cut_coefficients) {
                    models[t].addConstr(theta[t] >= coefficient[0] * (I[t - 1] + q_pre[t - 1]) +
                                                            coefficient[1] * ((1 + r0) * W0[t] -
                                                                              (1 + r1) * W1[t] -
                                                                              (1 + r2) * W2[t]) +
                                                            coefficient[2] * q[t] + coefficient[3]);
                }
            }

            for (int n = 0; n < forward_num; n++) {
                int index = scenarioPaths[n][t - 1];
                double demand = sample_details[t - 1][index];
                double rhs1 = t == 1 ? ini_I - demand
                                     : I_forward_values[iter][t - 2][n] +
                                               q_pre_values[iter][t - 2][n] - demand;
                if (t < T) {
                    double rhs2 = prices[t - 1] * demand +
                                  (1 + r0) * W0_forward_values[iter][t - 1][n] -
                                  (1 + r1) * W1_forward_values[iter][t - 1][n] -
                                  (1 + r2) * W2_forward_values[iter][t - 1][n];
                    double rhs3 = q_values[iter][t - 1][n];
                    models[t].setObjective(overhead_costs[t] + unit_vari_costs[t] * q[t] -
                                           prices[t - 1] * (demand - B[t - 1]) + r2 * W2[t] +
                                           r1 * W1[t] - r0 * W0[t] + theta[t]);
                    models[t].getConstr(1).set(GRB_DoubleAttr_RHS, rhs2);
                    models[t].getConstr(2).set(GRB_DoubleAttr_RHS, rhs3);
                    models[t].update();
                } else
                    models[t].setObjective(-prices[t - 1] * (demand - B[t - 1]) -
                                           unit_salvage_value * I[t - 1]);
                models[t].getConstr(0).set(GRB_DoubleAttr_RHS, rhs1);

                // optimize
                models[t].optimize();

                if (models[t].get(GRB_IntAttr_SolCount) == 1) {
                    I_forward_values[iter][t - 1][n] = I[t - 1].get(GRB_DoubleAttr_X);
                    if (t < T) {
                        q_values[iter][t][n] = q[t].get(GRB_DoubleAttr_X);
                        q_pre_values[iter][t - 1][n] = q_pre[t - 1].get(GRB_DoubleAttr_X);
                        W0_forward_values[iter][t][n] = W0[t].get(GRB_DoubleAttr_X);
                        W1_forward_values[iter][t][n] = W1[t].get(GRB_DoubleAttr_X);
                        W2_forward_values[iter][t][n] = W2[t].get(GRB_DoubleAttr_X);
                    }
                }
            }
        }

        // backward
        std::vector intercept_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope1_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope2_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope3_values(T, std::vector<std::vector<double>>(forward_num));
        for (int t = 0; t < T; t++) {
            for (int n = 0; n < forward_num; n++) {
                intercept_values[t][n].resize(sample_nums[t]);
                slope1_values[t][n].resize(sample_nums[t]);
                slope2_values[t][n].resize(sample_nums[t]);
                slope3_values[t][n].resize(sample_nums[t]);
            }
        }
        for (size_t t = T; t > 0; t--) {
            for (int n = 0; n < forward_num; n++) {
                size_t S = sample_details[t - 1].size();
                for (size_t s = 0; s < S; s++) {
                    auto demand = sample_details[t - 1][s];
                    double rhs1 = t == 1 ? ini_I - demand
                                         : I_forward_values[iter][t - 2][n] +
                                                   q_pre_values[iter][t - 2][n] - demand;
                    if (t < T) {
                        double rhs2 = prices[t - 1] * demand +
                                      (1 + r0) * W0_forward_values[iter][t - 1][n] -
                                      (1 + r1) * W1_forward_values[iter][t - 1][n] -
                                      (1 + r2) * W2_forward_values[iter][t - 1][n];
                        double rhs3 = q_values[iter][t - 1][n];
                        models[t].setObjective(overhead_costs[t] + unit_vari_costs[t] * q[t] -
                                               prices[t - 1] * (demand - B[t - 1]) + r2 * W2[t] +
                                               r1 * W1[t] - r0 * W0[t] + theta[t]);
                        models[t].getConstr(1).set(GRB_DoubleAttr_RHS, rhs2);
                        models[t].getConstr(2).set(GRB_DoubleAttr_RHS, rhs3);
                        models[t].update();
                    } else
                        models[t].setObjective(-prices[t - 1] * (demand - B[t - 1]) -
                                               unit_salvage_value * I[t - 1]);
                    models[t].getConstr(0).set(GRB_DoubleAttr_RHS, rhs1);

                    // optimize
                    models[t].optimize();

                    int pi_num = models[t].get(GRB_IntAttr_NumConstrs);
                    double pi[pi_num];
                    double rhs[pi_num];
                    for (int p = 0; p < pi_num; p++) {
                        GRBConstr constraint = models[t].getConstr(p);
                        pi[p] = constraint.get(GRB_DoubleAttr_Pi);
                        rhs[p] = constraint.get(GRB_DoubleAttr_RHS);
                    }
                    if (t < T) {
                        intercept_values[t - 1][n][s] += -pi[0] * demand +
                                                         pi[1] * prices[t - 1] * demand -
                                                         prices[t - 1] * demand + overhead_costs[t];
                    } else {
                        intercept_values[t - 1][n][s] += -pi[0] * demand - prices[t - 1] * demand;
                    }
                    for (size_t k = 3; k < pi_num; k++)
                        intercept_values[t - 1][n][s] += pi[k] * rhs[k];
                    slope1_values[t - 1][n][s] = pi[0];
                    if (t < T) {
                        slope2_values[t - 1][n][s] = pi[1];
                        slope3_values[t - 1][n][s] = pi[2];
                    }
                }
                double avg_intercept;
                double avg_slope1;
                double avg_slope2;
                double avg_slope3;
                for (size_t s = 0; s < S; s++) {
                    double sum = std::accumulate(intercept_values[t - 1][n].begin(),
                                                 intercept_values[t - 1][n].end(), 0.0);
                    avg_intercept = sum / static_cast<double>(S);
                    sum = std::accumulate(slope1_values[t - 1][n].begin(),
                                          slope1_values[t - 1][n].end(), 0.0);
                    avg_slope1 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope2_values[t - 1][n].begin(),
                                          slope2_values[t - 1][n].end(), 0.0);
                    avg_slope2 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope3_values[t - 1][n].begin(),
                                          slope3_values[t - 1][n].end(), 0.0);
                    avg_slope3 = sum / static_cast<double>(S);
                }
                slopes1[iter][t - 1][n] = avg_slope1;
                slopes2[iter][t - 1][n] = avg_slope2;
                slopes3[iter][t - 1][n] = avg_slope3;
                intercepts[iter][t - 1][n] = avg_intercept;
            }
        }
        iter = iter + 1;
    }

    double final_value = -models[0].get(GRB_DoubleAttr_ObjVal);
    double Q1 = q_values[iter - 1][0][0];
    return {final_value, Q1};
}

int main() {
    const SingleProduct single_product;
    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto result = single_product.solve();
    const auto end_time = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> diff = end_time - start_time;
    std::cout << "********************************************" << std::endl;
    std::cout << "cpu time is: " << diff.count() << " seconds" << std::endl;
    std::cout << "final expected cash balance is " << result[0] << std::endl;
    std::cout << "ordering Q in the first period is " << result[1] << std::endl;
    double optimal_value = 167.31;
    double gap = (result[0] - optimal_value) / optimal_value;
    std::cout << "gap is " << std::format("{: .2f}%", gap * 100) << std::endl;

    return 0;
}
