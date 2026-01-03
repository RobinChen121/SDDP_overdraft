/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#include "common.h"
#include <fstream>
#include <unordered_set>

std::string to_csv_line(const std::vector<std::string> &row) {
    std::string line;
    for (size_t i = 0; i < row.size(); ++i) {
        std::string cell = row[i];

        if (cell.find(',') != std::string::npos || cell.find('"') != std::string::npos) {
            size_t pos = 0;
            while ((pos = cell.find('"', pos)) != std::string::npos) {
                cell.insert(pos, "\"");
                pos += 2;
            }
            cell = "\"" + cell + "\"";
        }
        line += cell;
        if (i != row.size() - 1)
            line += ",";
    }
    return line;
}

void append_csv_head(const std::string &file_name, const std::string &head) {
    std::ofstream file(file_name, std::ios::app);
    file << head;
    file.close();
}


Matrix remove_duplicate_rows(const Matrix &mat) {
    std::unordered_set<std::vector<double>, VectorHash, VectorEqual> unique_rows(mat.begin(),
                                                                                 mat.end());
    return {unique_rows.begin(), unique_rows.end()};
}

PairStatus check_pair_status(const double end_inventory, const double end_cash,
                             const double overdraft_limit) {
    auto I_status = end_inventory > 0 ? IStatus::POSITIVE : IStatus::NEGATIVE;
    CashStatus cash_status;
    if (end_cash > 0) {
        cash_status = CashStatus::ATW0;
    } else if (end_cash < -overdraft_limit) {
        cash_status = CashStatus::ATW2;
    } else {
        cash_status = CashStatus::ATW1;
    }
    return {I_status, cash_status};
}

TripleStatus checkTripleStatus(const double end_inventory1, const double end_inventory2,
                               const double end_cash, const double overdraft_limit) {
    const auto I_status1 = end_inventory1 > 0 ? IStatus::POSITIVE : IStatus::NEGATIVE;
    const auto I_status2 = end_inventory2 > 0 ? IStatus::POSITIVE : IStatus::NEGATIVE;
    CashStatus cash_status;
    if (end_cash > 0) {
        cash_status = CashStatus::ATW0;
    } else if (end_cash < -overdraft_limit) {
        cash_status = CashStatus::ATW2;
    } else {
        cash_status = CashStatus::ATW1;
    }
    return {I_status1, I_status2, cash_status};
}

DoubleIStatus checkDoubleIStatus(const double end_inventory1, const double end_inventory2) {
    const auto I_status1 = end_inventory1 > 0 ? IStatus::POSITIVE : IStatus::NEGATIVE;
    const auto I_status2 = end_inventory2 > 0 ? IStatus::POSITIVE : IStatus::NEGATIVE;
    return {I_status1, I_status2};
}

double compute_ub_sigma(const std::vector<double> &ubs, double avg_ub) {
    const int K = ubs.size();
    double sigma_square = 0.0;
    for (int i = 0; i < K; i++) {
        sigma_square += pow(ubs[i] - avg_ub, 2);
    }
    sigma_square /= (K - 1);
    return std::sqrt(sigma_square);
}
