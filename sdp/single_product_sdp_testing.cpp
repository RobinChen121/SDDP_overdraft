/*
 * Created by Zhen Chen on 2025/12/28.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include <iostream>
#include <vector>
#include "single_product_sdp.h"
#include "../common.h"


int main() {
    const std::vector<std::vector<double>> demands_all = {
            {15, 15, 15, 15}, {25.0, 20.0, 10.0, 7.0}, {7, 12, 17, 23}, {24, 15, 5, 20},
            {5, 11, 22, 10},  {31, 14, 25, 12},        {40, 23, 8, 30}, {7, 30, 15, 12},
            {17, 6, 31, 22},  {9, 17, 35, 10}};

    const std::vector<double> overdraft_interests = {0.2, 0.15, 0.1, 0.05, 0.0};
    const std::vector<double> overhead_costs_all = {25.0, 50.0};
    const std::vector<double> prices_all = {5, 10};

    const std::string file_name = "../tests_results/sdp_single_prodcut_testing.csv";
    const std::string head =
            "demand pattern, interest rate, overhead, price, final value, time, Q\n";
    append_csv_head(file_name, head);

    for (int i = 0; i < demands_all.size(); i++) {
        for (int m = 0; m < overdraft_interests.size(); m++) {
            for (int j = 0; j < overhead_costs_all.size(); j++) {
                for (int k = 0; k < prices_all.size(); k++) {
                    const auto &demands = demands_all[i];
                    const double overhead = overhead_costs_all[j];
                    const double price = prices_all[k];
                    const double interest = overdraft_interests[m];

                    auto problem =
                            OverdraftLeadtimeSingleProduct(demands, interest, overhead, price);
                    const auto start_time = std::chrono::high_resolution_clock::now();
                    const auto final_value = problem.solve();
                    const auto end_time = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> time = end_time - start_time;
                    const double Q = problem.solve()[1];
                    std::vector<double> arr = {
                            static_cast<double>(i), interest,     overhead, price,
                            final_value[0],         time.count(), Q};
                    append_csv_row(file_name, head);
                    std::cout << "**************************************************" << std::endl;
                    std::cout << "running time is " << time.count() << 's' << std::endl;
                    std::cout << "Final expected cash increment is " << final_value[0] << std::endl;
                    std::cout << "Optimal Q in the first period is " << final_value[1] << std::endl;
                }
            }
        }
    }

    return 0;
}
