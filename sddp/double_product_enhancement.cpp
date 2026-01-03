/*
 * Created by Zhen Chen on 2025/12/30.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "double_product_enhancement.h"
#include <numeric>
#include <unordered_set>
#include "../common.h"
#include "gurobi_c++.h"
#include "sampling.h"

std::array<double, 3> DoubleProduct::solve() const {

    const std::vector<int> sample_nums(T, sample_num);
    std::vector<std::vector<double>> sample_details1(T);
    std::vector<std::vector<double>> sample_details2(T);

    // gurobi environments and model
    GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    std::vector<GRBModel> models(T + 1, GRBModel(env));

    // decision variables
    std::vector<GRBVar> q1(T);
    std::vector<GRBVar> q2(T);
    std::vector<GRBVar> q1_pre(T - 1);
    std::vector<GRBVar> q2_pre(T - 1);
    std::vector<GRBVar> theta(T);
    std::vector<GRBVar> I1(T);
    std::vector<GRBVar> I2(T);
    std::vector<GRBVar> B1(T);
    std::vector<GRBVar> B2(T);
    std::vector<GRBVar> cash(T);
    std::vector<GRBVar> W0(T);
    std::vector<GRBVar> W1(T);
    std::vector<GRBVar> W2(T);

    // build initial models for each stage
    for (int t = 0; t < T + 1; t++) {
        // vars
        if (t > 0) {
            I1[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "I1_" + std::to_string(t));
            I2[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "I2_" + std::to_string(t));
            B1[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "B1_" + std::to_string(t));
            B2[t - 1] =
                    models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "B2_" + std::to_string(t));
            if (t < T) {
                cash[t - 1] = models[t].addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                               "cash_" + std::to_string(t));
                q1_pre[t - 1] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                                 "q1_pre" + std::to_string(t + 1));
                q2_pre[t - 1] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                                 "q2_pre" + std::to_string(t + 1));
            }
        }
        if (t < T) {
            q1[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "q1_" + std::to_string(t + 1));
            q2[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "q2_" + std::to_string(t + 1));
            W0[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W0_" + std::to_string(t + 1));
            W1[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W1_" + std::to_string(t + 1));
            W2[t] = models[t].addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                     "W2_" + std::to_string(t + 1));
            theta[t] = models[t].addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS,
                                        "theta_" + std::to_string(t + 2));
        }
        // constraints
        if (t > 0) {
            models[t].addConstr(I1[t - 1] - B1[t - 1] == 0);
            models[t].addConstr(I2[t - 1] - B2[t - 1] == 0);
            if (t < T) {
                models[t].addConstr(
                        cash[t - 1] + prices1[t - 1] * B1[t - 1] + prices2[t - 1] * B2[t - 1] == 0);
                models[t].addConstr(q1_pre[t - 1] == 0);
                models[t].addConstr(q2_pre[t - 1] == 0);
            }
        }
        if (t < T) {
            models[t].addConstr(W1[t] <= overdraft_limit);
            if (t == 0)
                models[t].addConstr(ini_cash - unit_vari_order_costs1[t] * q1[t] -
                                            unit_vari_order_costs2[t] * q2[t] - W0[t] + W1[t] +
                                            W2[t] ==
                                    overhead_costs[t]);
            else {
                models[t].addConstr(cash[t - 1] - unit_vari_order_costs1[t] * q1[t] -
                                            unit_vari_order_costs2[t] * q2[t] - W0[t] + W1[t] +
                                            W2[t] ==
                                    overhead_costs[t]);
            }
            models[t].addConstr(theta[t] >= theta_initial_value * (static_cast<double>(T) - t));
        }
        if (t == 0) {
            models[t].setObjective(overhead_costs[0] + unit_vari_order_costs1[0] * q1[0] +
                                   unit_vari_order_costs2[0] * q2[0] + r2 * W2[0] + r1 * W1[0] -
                                   r0 * W0[0] + theta[0]);
        }
        models[t].update();
    }

    std::vector intercepts(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector slopes3_1(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector slopes3_2(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector slopes1_1(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector slopes1_2(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector slopes2(iter_num, std::vector(T, std::vector<double>(forward_num)));

    std::vector q1_pre_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector q2_pre_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector q1_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector q2_values(iter_num, std::vector(T, std::vector<double>(forward_num)));

    std::vector I1_forward_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector I2_forward_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector W0_forward_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector W1_forward_values(iter_num, std::vector(T, std::vector<double>(forward_num)));
    std::vector W2_forward_values(iter_num, std::vector(T, std::vector<double>(forward_num)));

    // no duplicate cuts during iteration
    std::vector<std::unordered_set<std::vector<double>, VectorHash>> cut_coefficients_cache(T);

    for (int t = 0; t < T; t++) { // better in the front of the while loop
        sample_details1[t].resize(sample_nums[t]);
        sample_details2[t].resize(sample_nums[t]);

        sample_details1[t] = generate_samples_poisson(sample_nums[t], mean_demand1[t]);
        sample_details2[t] = generate_samples_poisson(sample_nums[t], mean_demand2[t]);
    }

    int iter = 0;
    while (iter < iter_num) {

        auto scenario_paths1 = generate_scenario_paths(forward_num, sample_nums);
        auto scenario_paths2 = generate_scenario_paths(forward_num, sample_nums);

        if (iter > 0) {
            if (iter == 1) { // remove the big M constraints at iteration 2
                int index = models[0].get(GRB_IntAttr_NumConstrs) - 1;
                models[0].remove(models[0].getConstr(index));
            }

            std::vector<double> this_coefficients = {
                    slopes1_1[iter - 1][0][0], slopes1_2[iter - 1][0][0],
                    slopes2[iter - 1][0][0],   slopes3_1[iter - 1][0][0],
                    slopes3_2[iter - 1][0][0], intercepts[iter - 1][0][0]};

            if (cut_coefficients_cache[0].empty() ||
                !cut_coefficients_cache[0].contains(this_coefficients)) {
                models[0].addConstr(
                        theta[0] >=
                        this_coefficients[0] * iniI + this_coefficients[1] * iniI +
                                this_coefficients[2] *
                                        ((1 + r0) * W0[0] - (1 + r1) * W1[0] - (1 + r2) * W2[0]) +
                                this_coefficients[3] * q1[0] + this_coefficients[4] * q2[0] +
                                this_coefficients[5]);
                models[0].update();
                cut_coefficients_cache[0].emplace(this_coefficients);
            }
        }
        models[0].optimize();

        for (int n = 0; n < forward_num; n++) {
            q1_values[iter][0][n] = q1[0].get(GRB_DoubleAttr_X);
            q2_values[iter][0][n] = q2[0].get(GRB_DoubleAttr_X);
            W0_forward_values[iter][0][n] = W0[0].get(GRB_DoubleAttr_X);
            W1_forward_values[iter][0][n] = W1[0].get(GRB_DoubleAttr_X);
            W2_forward_values[iter][0][n] = W2[0].get(GRB_DoubleAttr_X);
        }

        // forward
        for (int t = 1; t < T + 1; t++) {
            if (iter == 1 and t < T) { // remove the big M constraints at iteration 2
                int index = models[t].get(GRB_IntAttr_NumConstrs) - 1;
                models[t].remove(models[t].getConstr(index));
            }

            if (iter > 0 && t < T) {
                std::vector<std::vector<double>> cut_coefficients(forward_num,
                                                                  std::vector<double>(6, 0));
                for (int n = 0; n < forward_num; n++) {
                    cut_coefficients[n][0] = slopes1_1[iter - 1][t][n];
                    cut_coefficients[n][1] = slopes1_2[iter - 1][t][n];
                    cut_coefficients[n][2] = slopes2[iter - 1][t][n];
                    cut_coefficients[n][3] = slopes3_1[iter - 1][t][n];
                    cut_coefficients[n][4] = slopes3_2[iter - 1][t][n];
                    cut_coefficients[n][5] = intercepts[iter - 1][t][n];
                };

                for (auto coefficients = remove_duplicate_rows(cut_coefficients); // cutCoefficients
                     auto &final_coefficient: coefficients) {
                    if (cut_coefficients_cache[t].empty() ||
                        !cut_coefficients_cache[t].contains(final_coefficient)) {
                        cut_coefficients_cache[t].emplace(final_coefficient);
                        models[t].addConstr(
                                theta[t] >=
                                final_coefficient[0] * (I1[t - 1] + q1_pre[t - 1]) +
                                        final_coefficient[1] * (I2[t - 1] + q2_pre[t - 1]) +
                                        final_coefficient[2] *
                                                ((1 + r0) * W0[t] - (1 + r1) * W1[t] -
                                                 (1 + r2) * W2[t]) +
                                        final_coefficient[3] * q1[t] +
                                        final_coefficient[4] * q2[t] + final_coefficient[5]);
                    }
                }
            }
            for (int n = 0; n < forward_num; n++) {
                int index1 = scenario_paths1[n][t - 1];
                int index2 = scenario_paths2[n][t - 1];
                double demand1 = sample_details1[t - 1][index1];
                double demand2 = sample_details2[t - 1][index2];
                double rhs1_1 = t == 1 ? iniI - demand1
                                       : I1_forward_values[iter][t - 2][n] +
                                                 q1_pre_values[iter][t - 2][n] - demand1;
                double rhs1_2 = t == 1 ? iniI - demand2
                                       : I2_forward_values[iter][t - 2][n] +
                                                 q2_pre_values[iter][t - 2][n] - demand2;
                if (t < T) {
                    double rhs2 = prices1[t - 1] * demand1 + prices2[t - 1] * demand2 +
                                  (1 + r0) * W0_forward_values[iter][t - 1][n] -
                                  (1 + r1) * W1_forward_values[iter][t - 1][n] -
                                  (1 + r2) * W2_forward_values[iter][t - 1][n];
                    double rhs3_1 = q1_values[iter][t - 1][n];
                    double rhs3_2 = q2_values[iter][t - 1][n];
                    models[t].setObjective(overhead_costs[t] + unit_vari_order_costs1[t] * q1[t] +
                                           unit_vari_order_costs2[t] * q2[t] -
                                           prices1[t - 1] * (demand1 - B1[t - 1]) -
                                           prices2[t - 1] * (demand2 - B2[t - 1]) + r2 * W2[t] +
                                           r1 * W1[t] - r0 * W0[t] + theta[t]);
                    models[t].getConstr(2).set(GRB_DoubleAttr_RHS, rhs2);
                    models[t].getConstr(3).set(GRB_DoubleAttr_RHS, rhs3_1);
                    models[t].getConstr(4).set(GRB_DoubleAttr_RHS, rhs3_2);
                    models[t].update();
                } else {
                    models[t].setObjective(-prices1[t - 1] * (demand1 - B1[t - 1]) -
                                           prices2[t - 1] * (demand2 - B2[t - 1]) -
                                           unit_salvage_value1 * I1[t - 1] -
                                           unit_salvage_value2 * I2[t - 1]);
                }
                models[t].getConstr(0).set(GRB_DoubleAttr_RHS, rhs1_1);
                models[t].getConstr(1).set(GRB_DoubleAttr_RHS, rhs1_2);

                // optimize
                try {
                    models[t].optimize();

                    I1_forward_values[iter][t - 1][n] = I1[t - 1].get(GRB_DoubleAttr_X);
                    I2_forward_values[iter][t - 1][n] = I2[t - 1].get(GRB_DoubleAttr_X);
                    if (t < T) {
                        q1_values[iter][t][n] = q1[t].get(GRB_DoubleAttr_X);
                        q2_values[iter][t][n] = q2[t].get(GRB_DoubleAttr_X);
                        q1_pre_values[iter][t - 1][n] = q1_pre[t - 1].get(GRB_DoubleAttr_X);
                        q2_pre_values[iter][t - 1][n] = q2_pre[t - 1].get(GRB_DoubleAttr_X);
                        W0_forward_values[iter][t][n] = W0[t].get(GRB_DoubleAttr_X);
                        W1_forward_values[iter][t][n] = W1[t].get(GRB_DoubleAttr_X);
                        W2_forward_values[iter][t][n] = W2[t].get(GRB_DoubleAttr_X);
                    }
                } catch (const GRBException &e) {
                    std::cout << e.getErrorCode() << std::endl;
                    std::cout << e.getMessage() << std::endl;
                }
            }
        }
        // backward
        std::vector intercept_back_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope1_1back_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope1_2back_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope2_back_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope3_1back_values(T, std::vector<std::vector<double>>(forward_num));
        std::vector slope3_2back_values(T, std::vector<std::vector<double>>(forward_num));

        for (size_t t = T; t > 0; t--) {
            auto sample_details = cartesian_product(sample_details1[t - 1], sample_details2[t - 1]);
            for (int n = 0; n < forward_num; n++) {
                size_t S = sample_details.size();

                intercept_back_values[t - 1][n].resize(S);
                slope1_1back_values[t - 1][n].resize(S);
                slope1_2back_values[t - 1][n].resize(S);
                slope2_back_values[t - 1][n].resize(S);
                slope3_1back_values[t - 1][n].resize(S);
                slope3_2back_values[t - 1][n].resize(S);
                for (size_t s = 0; s < S; s++) {
                    auto demand1 = sample_details[s].first;
                    auto demand2 = sample_details[s].second;
                    double rhs1_1 = t == 1 ? iniI - demand1
                                           : I1_forward_values[iter][t - 2][n] +
                                                     q1_pre_values[iter][t - 2][n] - demand1;
                    double rhs1_2 = t == 1 ? iniI - demand2
                                           : I2_forward_values[iter][t - 2][n] +
                                                     q2_pre_values[iter][t - 2][n] - demand2;
                    if (t < T) {
                        double rhs2 = prices1[t - 1] * demand1 + prices2[t - 1] * demand2 +
                                      (1 + r0) * W0_forward_values[iter][t - 1][n] -
                                      (1 + r1) * W1_forward_values[iter][t - 1][n] -
                                      (1 + r2) * W2_forward_values[iter][t - 1][n];
                        double rhs3_1 = q1_values[iter][t - 1][n];
                        double rhs3_2 = q2_values[iter][t - 1][n];
                        models[t].setObjective(overhead_costs[t] +
                                               unit_vari_order_costs1[t] * q1[t] +
                                               unit_vari_order_costs2[t] * q2[t] -
                                               prices1[t - 1] * (demand1 - B1[t - 1]) -
                                               prices2[t - 1] * (demand2 - B2[t - 1]) + r2 * W2[t] +
                                               r1 * W1[t] - r0 * W0[t] + theta[t]);
                        models[t].getConstr(2).set(GRB_DoubleAttr_RHS, rhs2);
                        models[t].getConstr(3).set(GRB_DoubleAttr_RHS, rhs3_1);
                        models[t].getConstr(4).set(GRB_DoubleAttr_RHS, rhs3_2);
                        models[t].update();

                    } else {
                        models[t].setObjective(-prices1[t - 1] * (demand1 - B1[t - 1]) -
                                               prices2[t - 1] * (demand2 - B2[t - 1]) -
                                               unit_salvage_value1 * I1[t - 1] -
                                               unit_salvage_value2 * I2[t - 1]);
                    }
                    models[t].getConstr(0).set(GRB_DoubleAttr_RHS, rhs1_1);
                    models[t].getConstr(1).set(GRB_DoubleAttr_RHS, rhs1_2);

                    // optimize
                    try {
                        models[t].optimize();
                    } catch (const GRBException &e) {
                        std::cout << e.getErrorCode() << std::endl;
                        std::cout << e.getMessage() << std::endl;
                    }
                    int piNum = models[t].get(GRB_IntAttr_NumConstrs);
                    double pi[piNum];
                    double rhs[piNum];

                    for (int p = 0; p < piNum; p++) {
                        GRBConstr constraint = models[t].getConstr(p);
                        pi[p] = constraint.get(GRB_DoubleAttr_Pi);
                        rhs[p] = constraint.get(GRB_DoubleAttr_RHS);
                    }
                    if (t < T) {
                        intercept_back_values[t - 1][n][s] +=
                                -pi[0] * demand1 - pi[1] * demand2 +
                                pi[2] * prices1[t - 1] * demand1 +
                                pi[2] * prices2[t - 1] * demand2 - prices1[t - 1] * demand1 -
                                prices2[t - 1] * demand2 + overhead_costs[t];
                    } else {
                        intercept_back_values[t - 1][n][s] += -pi[0] * demand1 - pi[1] * demand2 -
                                                              prices1[t - 1] * demand1 -
                                                              prices2[t - 1] * demand2;
                    }
                    for (size_t k = 5; k < piNum; k++)
                        intercept_back_values[t - 1][n][s] += pi[k] * rhs[k];
                    slope1_1back_values[t - 1][n][s] = pi[0];
                    slope1_2back_values[t - 1][n][s] = pi[1];
                    if (t < T) {
                        slope2_back_values[t - 1][n][s] = pi[2];
                        slope3_1back_values[t - 1][n][s] = pi[3];
                        slope3_2back_values[t - 1][n][s] = pi[4];
                    }
                }

                double avg_intercept;
                double avg_slope1_1;
                double avg_slope1_2;
                double avg_slope2;
                double avg_slope3_1;
                double avg_slope3_2;
                for (size_t s = 0; s < S; s++) {
                    double sum = std::accumulate(intercept_back_values[t - 1][n].begin(),
                                                 intercept_back_values[t - 1][n].end(), 0.0);
                    avg_intercept = sum / static_cast<double>(S);
                    sum = std::accumulate(slope1_1back_values[t - 1][n].begin(),
                                          slope1_1back_values[t - 1][n].end(), 0.0);
                    avg_slope1_1 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope1_2back_values[t - 1][n].begin(),
                                          slope1_2back_values[t - 1][n].end(), 0.0);
                    avg_slope1_2 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope2_back_values[t - 1][n].begin(),
                                          slope2_back_values[t - 1][n].end(), 0.0);
                    avg_slope2 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope3_1back_values[t - 1][n].begin(),
                                          slope3_1back_values[t - 1][n].end(), 0.0);
                    avg_slope3_1 = sum / static_cast<double>(S);
                    sum = std::accumulate(slope3_2back_values[t - 1][n].begin(),
                                          slope3_2back_values[t - 1][n].end(), 0.0);
                    avg_slope3_2 = sum / static_cast<double>(S);
                }
                intercepts[iter][t - 1][n] = avg_intercept;
                slopes1_1[iter][t - 1][n] = avg_slope1_1;
                slopes1_2[iter][t - 1][n] = avg_slope1_2;
                slopes2[iter][t - 1][n] = avg_slope2;
                slopes3_1[iter][t - 1][n] = avg_slope3_1;
                slopes3_2[iter][t - 1][n] = avg_slope3_2;
            }
        }

        // std::cout << "iteration " << iter << ", objective is " << std::fixed <<
        // std::setprecision(2)
        //           << -models[0].get(GRB_DoubleAttr_ObjVal) << std::endl;
        iter++;
    }

    // std::cout << "********************************************" << std::endl;
    // std::cout << "after " << iter << " iterations, sample number " << sample_num
    //           << " and scenario number " << forward_num << std::endl;

    double final_value = -models[0].get(GRB_DoubleAttr_ObjVal);
    double Q1 = q1_values[iter - 1][0][0];
    double Q2 = q2_values[iter - 1][0][0];
    return {final_value, Q1, Q2};
}

int main() {
    const auto problem = DoubleProduct();
    const auto start = std::chrono::high_resolution_clock::now();
    const auto result = problem.solve();
    const auto end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> duration = end - start;
    std::cout << "running time is " << duration.count() << 's' << std::endl;
    std::cout << "final expected cash balance is " << result[0] << std::endl;
    std::cout << "ordering Q1 in the first period is " << result[1] << std::endl;
    std::cout << "ordering Q2 in the first period is " << result[2] << std::endl;
    return 0;
}
