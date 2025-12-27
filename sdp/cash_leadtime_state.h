/*
 * Created by Zhen Chen on 2025/12/27.
 * Email: chen.zhen5526@gmail.com
 * Description:
 *
 *
 */

#ifndef CASH_LEADTIME_STATE_H
#define CASH_LEADTIME_STATE_H

#include <boost/functional/hash.hpp>

class CashLeadtimeState {
    int period = 1;
    double ini_inventory{};
    double ini_cash{};
    double q_pre{}; // ordering quantity in the last period

public:
    CashLeadtimeState(const int period, const double ini_inventory, const double ini_cash,
                      const double q_pre) :
        period(period), ini_inventory(ini_inventory), ini_cash(ini_cash), q_pre(q_pre) {};

    int get_period() const { return period; }
    double get_inventory() const { return ini_inventory; }
    double get_cash() const { return ini_cash; }
    double get_pre() const { return q_pre; }

    // for ordered map
    bool operator==(const CashLeadtimeState &other) const {
        return period == other.period && ini_inventory == other.ini_inventory &&
               ini_cash == other.ini_cash && q_pre == other.q_pre;
    }

    // hash
    friend struct std::hash<CashLeadtimeState>;
};

template<>
struct std::hash<CashLeadtimeState> {
    std::size_t operator()(const CashLeadtimeState &s) const noexcept {
        std::size_t seed = 0;
        boost::hash_combine(seed, s.get_period());
        boost::hash_combine(seed, s.get_inventory());
        boost::hash_combine(seed, s.get_cash());
        boost::hash_combine(seed, s.get_pre());
    }
};

#endif // CASH_LEADTIME_STATE_H
