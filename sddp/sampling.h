/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef SAMPLING_H
#define SAMPLING_H
#include <vector>

std::vector<double> generate_samples_poisson(int sample_nums, double mean);

int rand_uniform(int a, int b);

std::vector<std::vector<int>> generate_scenario_paths(int scenario_num,
                                                      const std::vector<int> &sample_nums);
#endif // SAMPLING_H
