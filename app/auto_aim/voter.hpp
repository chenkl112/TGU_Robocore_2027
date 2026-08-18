#ifndef TGU_ROBOCORE_2027_VOTER_HPP
#define TGU_ROBOCORE_2027_VOTER_HPP
#pragma once

#include <vector>

#include "armor.hpp"

namespace app::auto_aim {

class Voter {
public:
    Voter();
    void vote(const Color color, const ArmorName name, const ArmorType type);
    std::size_t count(const Color color, const ArmorName name, const ArmorType type);

private:
    std::vector<std::size_t> count_;
    std::size_t index(const Color color, const ArmorName name, const ArmorType type) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_VOTER_HPP
