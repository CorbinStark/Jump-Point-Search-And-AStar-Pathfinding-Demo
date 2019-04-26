#include <random>

static inline
i32 random_int(i32 min, i32 max) {
    static std::random_device rd;
    static std::mt19937 mt(rd());
    std::uniform_int_distribution<> dist(0, max-min);
    return dist(mt) + min;
}
