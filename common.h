/*
 * Created by Zhen Chen on 2025/12/29.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef COMMON_H
#define COMMON_H

#include <boost/functional/hash.hpp>
#include <fstream>
#include <iostream>
#include <vector>

void append_csv_head(const std::string &file_name, const std::string &head);

std::string to_csv_line(const std::vector<std::string> &row);

template<typename T>
void append_csv_row(const std::string &filename, const std::vector<T> &row_data) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "unable to open the file!" << filename << std::endl;
        return;
    }

    std::vector<std::string> string_rows;
    for (const auto &item: row_data) {
        string_rows.push_back(toString(item));
    }

    file << to_csv_line(string_rows) << "\n";
    file.close();
}


using Matrix = std::vector<std::vector<double>>;
Matrix remove_duplicate_rows(const Matrix &mat);

struct VectorHash {
    size_t operator()(const std::vector<double> &v) const {
        std::size_t seed = 0;
        for (const double num: v) {
            boost::hash_combine(seed, num);
        }
        return seed;
    }
};

struct VectorEqual {
    bool operator()(const std::vector<double> &a, const std::vector<double> &b) const {
        if (a.size() != b.size())
            return false;
        for (size_t i = 0; i < a.size(); ++i)
            if (std::abs(a[i] - b[i]) > 1e-4)
                return false;
        return true;
    }
};

enum class IStatus { POSITIVE, NEGATIVE };

enum class CashStatus { ATW0, ATW1, ATW2 };

using PairStatus = std::pair<IStatus, CashStatus>;

using DoubleIStatus = std::pair<IStatus, IStatus>;

struct TripleStatus {
    IStatus I_status1;
    IStatus I_status2;
    CashStatus cash_status;

    bool operator==(const TripleStatus &other) const {
        return I_status1 == other.I_status1 && I_status2 == other.I_status2 &&
               cash_status == other.cash_status;
    }
};

template<>
struct std::hash<PairStatus> {
    std::size_t operator()(const PairStatus &p) const {
        const size_t first_hash = std::hash<IStatus>{}(p.first);
        const size_t second_hash = std::hash<CashStatus>{}(p.second);
        return first_hash ^ (second_hash << 1);
    }
};

PairStatus check_pair_status(double end_inventory, double end_cash, double overdraft_limit);
#endif // COMMON_H
