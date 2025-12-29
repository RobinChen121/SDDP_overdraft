/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "sampling.h"
#include <random>
#include "../sdp/pmf.h"

std::vector<double> generate_samples_poisson(const int sample_nums, const double mean) {
    std::vector<double> samples(sample_nums);
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < sample_nums; ++i) {
        // LHS sampling
        std::uniform_real_distribution<> dis(static_cast<double>(i) / sample_nums,
                                             (i + 1.0) / sample_nums);
        const double random_value = dis(gen);
        samples[i] = poisson_quantile(random_value, mean);
    }
    return samples;
}

int rand_uniform(const int a, const int b) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<int> dist(a, b);
    const int random_number = dist(gen);
    return random_number;
}

std::vector<std::vector<int>> generate_scenario_paths(const int scenario_num,
                                                      const std::vector<int> &sample_nums) {
    const size_t T = sample_nums.size();
    std::vector<std::vector<int>> scenario_paths(scenario_num);
    for (int i = 0; i < scenario_num; ++i) {
        scenario_paths[i].resize(T);
    }

    for (int i = 0; i < scenario_num; ++i) {
        for (int t = 0; t < T; ++t) {
            scenario_paths[i][t] = rand_uniform(0, sample_nums[t] - 1);
        }
    }
    return scenario_paths;
}
