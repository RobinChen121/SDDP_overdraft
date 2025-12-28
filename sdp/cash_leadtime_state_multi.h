/*
 * Created by Zhen Chen on 2025/12/28.
 * Email: chen.zhen5526@gmail.com
 * Description: 
 *
 *
 */

#ifndef CASH_LEADTIME_STATE_MULTI_H
#define CASH_LEADTIME_STATE_MULTI_H

#include <boost/functional/hash.hpp>

class CashLeadtimeMultiState {
private:
    int period{};
    double ini_I1{};
    double ini_I2{};
    double q_pre1{};
    double q_pre2{};
    double ini_cash{};

public:
    CashLeadtimeMultiState(const int period, const double ini_I1, const double ini_I2,
                           const double q_pre1, const double q_pre2, const double ini_cash)
        : period(period), ini_I1(ini_I1), ini_I2(ini_I2), q_pre1(q_pre1), q_pre2(q_pre2),
          ini_cash(ini_cash) {};

    bool operator==(const CashLeadtimeMultiState &other) const{
        return period == other.period && ini_I1 == other.ini_I1 && ini_I2 == other.ini_I2 &&
         q_pre1 == other.q_pre1 && q_pre2 == other.q_pre2 && ini_cash == other.ini_cash;
    };


    friend struct std::hash<CashLeadtimeMultiState>;

    [[nodiscard]] int get_period() const { return period; }
    [[nodiscard]] double get_iniI1() const { return ini_I1; }
    [[nodiscard]] double get_iniI2() const { return ini_I2; }
    [[nodiscard]] double get_q_pre1() const { return q_pre1; }
    [[nodiscard]] double get_q_pre2() const { return q_pre2; }
    [[nodiscard]] double get_ini_cash() const { return ini_cash; }
};

template <>
struct std::hash<CashLeadtimeMultiState> {
    size_t operator()(const CashLeadtimeMultiState &s) const noexcept {
        std::size_t seed = 0;
        boost::hash_combine(seed, s.period);
        boost::hash_combine(seed, s.ini_I1);
        boost::hash_combine(seed, s.ini_I2);
        boost::hash_combine(seed, s.q_pre1);
        boost::hash_combine(seed, s.q_pre2);
        boost::hash_combine(seed, s.ini_cash);
        return seed;
    }
};

#endif //CASH_LEADTIME_STATE_MULTI_H
