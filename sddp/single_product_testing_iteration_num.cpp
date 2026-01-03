/*
 * Created by Zhen Chen on 2025/12/30.
 * Email: chen.zhen5526@gmail.com
 * Description: 
 *
 *
 */

/*
 * Created by Zhen Chen on 2025/4/28.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *  test the influence of iteration num.
 *
 *
 */

#include "single_product_enhancement.h"
#include <vector>
#include "../common.h"

int main() {
  std::vector<double> demands = {15, 15, 15, 15};

  double interest = 0.1;
  double overhead = 50;
  double price = 10;

  int sampleNum = 10;
  int forwardNum = 30;
  std::vector<int> iter_nums = {10};

  int runs = 20;
  const std::string file_name =
      "../tests_result/sddp_singleproduct_iterNum_testing.csv";
  const std::string head = "run,final value, "
                           "time,Q,sample number,forward number,iter number,gap\n";
  append_csv_head(file_name, head);

  double opt = 167.38;
  for (int i = 0; i < 1; i++) {
    for (int iter_num : iter_nums) {
      for (int n = 0; n < runs; n++) {
        //            const auto &demands = demands_all[i];
        auto problem =
            SingleProduct(demands, price, interest, overhead, sampleNum, forwardNum, iter_num);
        const auto start_time = std::chrono::high_resolution_clock::now();
        auto result = problem.solve();
        const auto end_time = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double> time = end_time - start_time;

        const double final_value = result[0];
        double gap = final_value - opt;
        const double Q = result[1];
        std::vector arr = {static_cast<double>(n),
                           final_value,
                           time.count(),
                           Q,
                           static_cast<double>(sampleNum),
                           static_cast<double>(forwardNum),
                           static_cast<double>(iter_num),
                           gap};
        append_csv_row(file_name, arr);
        std::cout << "**************************************************" << std::endl;
        std::cout << "running time is " << time.count() << 's' << std::endl;
        std::cout << "Final expected cash increment is " << final_value << std::endl;
        std::cout << "Optimal Q in the first period is " << Q << std::endl;
        std::cout << std::endl;
      }
    }
  }

  return 0;
}