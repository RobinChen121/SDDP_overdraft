/*
 * Created by Zhen Chen on 2025/12/30.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include <vector>
#include "../common.h"
#include "double_product_enhancement_further.h"

int main() {
    std::vector<std::vector<double>> demands_all = {
            {30, 30, 30, 30, 30, 30},    {50, 46, 38, 28, 23, 18},  {14, 18, 23, 33, 42, 49},
            {47, 30, 13, 30, 47, 54},    {21, 24, 39, 30, 24, 18},  {63, 10, 4, 33, 67, 14},
            {15, 140, 147, 74, 109, 88}, {14, 71, 49, 152, 78, 33}, {13, 35, 79, 43, 44, 59},
            {15, 56, 19, 84, 136, 67}};
    const std::vector overdraft_interests = {0.2, 0.15, 0.1, 0.05, 0.0};
    std::vector overdraft_limits = {300, 400, 500, 600, 700};
    double overdraft_limit = 500;
    double overdraft_interest = 0.1;

    int sampleNum = 20;
    int forwardNum = 10;
    int iterNum = 100;
    int skip_num = 11;

    int runs = 1;
    const std::string file_name = "../test_result/sddp_doubleproduct_SKIP_testing.csv";
    const std::string head = "run,demand pattern,interest rate,overdraft_limit,final value,"
                             "time,Q1,Q2,sample number,forward number,iter number\n";
    append_csv_head(file_name, head);

    for (int i = 0; i < demands_all.size(); i++) {
        for (int n = 0; n < runs; n++) {
            const auto &demands = demands_all[i];
            auto problem = DoubleProduct(demands, overdraft_interest, overdraft_limit, sampleNum,
                                         forwardNum, iterNum);
            const auto start_time = std::chrono::high_resolution_clock::now();
            auto result = problem.solve();
            const double final_value = result[0];
            const auto end_time = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> time = end_time - start_time;

            double Q1 = result[1];
            double Q2 = result[2];
            std::vector arr = {static_cast<double>(n),
                               static_cast<double>(i),
                               overdraft_interest,
                               overdraft_limit,
                               final_value,
                               time.count(),
                               Q1,
                               Q2,
                               static_cast<double>(sampleNum),
                               static_cast<double>(forwardNum),
                               static_cast<double>(iterNum),
                               static_cast<double>(skip_num)};
            append_csv_row(file_name, arr);
            // std::cout << "**************************************************" << std::endl;
            std::cout << "running time is " << time.count() << 's' << std::endl;
            std::cout << "Final expected cash increment is " << final_value << std::endl;
            std::cout << "Optimal Q1 in the first period is " << Q1 << std::endl;
            std::cout << "Optimal Q2 in the first period is " << Q2 << std::endl;
            std::cout << std::endl;
            // }
        }
    }

    return 0;
}
