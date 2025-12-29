/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include <vector>
#include "../common.h"
#include "single_product_enhancement.h"

int main() {
    std::vector<std::vector<double>> demands_all = {
            {15, 15, 15, 15}, {25.0, 20.0, 10.0, 7.0}, {7, 12, 17, 23}, {24, 15, 5, 20},
            {5, 11, 22, 10},  {31, 14, 25, 12},        {40, 23, 8, 30}, {7, 30, 15, 12},
            {17, 6, 31, 22},  {9, 17, 35, 10}};

    const std::vector overdraft_interests = {0.2, 0.15, 0.1, 0.05, 0.0};
    std::vector overhead_costs_all = {25.0, 50.0};
    std::vector prices_all = {5, 10};

    int sample_num = 10;
    int forward_num = 30;
    int iter_num = 50;

    int skip_num = 3; // l

    int runs = 20;

    const std::string file_name = "../test_results/sddp_singleproduct_enhance_testing.csv";
    const std::string head = "run,demand pattern,interest rate,overhead,price,final value, "
                             "time,Q,sample number,forward number,iter number, skip num\n";
    append_csv_head(file_name, head);

    for (int i = 0; i < demands_all.size(); i++) {
        for (double interest: overdraft_interests) {
            for (double overhead: overhead_costs_all) {
                for (double price: prices_all) {
                    for (int n = 0; n < runs; n++) {
                        const auto &demands = demands_all[i];
                        auto problem = SingleProduct(demands, price, interest, overhead, sample_num,
                                                     forward_num, iter_num);
                        const auto start_time = std::chrono::high_resolution_clock::now();
                        auto result = problem.solve();
                        const double final_value = result[0];
                        const auto end_time = std::chrono::high_resolution_clock::now();
                        const std::chrono::duration<double> time = end_time - start_time;

                        const double Q = result[1];
                        std::vector arr = {static_cast<double>(n),
                                           static_cast<double>(i),
                                           interest,
                                           overhead,
                                           price,
                                           final_value,
                                           time.count(),
                                           Q,
                                           static_cast<double>(sample_num),
                                           static_cast<double>(forward_num),
                                           static_cast<double>(iter_num)};
                        append_csv_row(file_name, arr);
                        std::cout << "**************************************************"
                                  << std::endl;
                        std::cout << "running time is " << time.count() << 's' << std::endl;
                        std::cout << "Final expected cash increment is " << final_value
                                  << std::endl;
                        std::cout << "Optimal Q in the first period is " << Q << std::endl;
                        std::cout << std::endl;
                    }
                }
            }
        }
    }

    return 0;
}
