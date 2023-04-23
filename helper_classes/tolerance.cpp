#include <cmath>

#include "tolerance.h"

using namespace Symmetry;


double Difference::difference(const double v1, const double v2) {
    return std::abs(v1 - v2);
}


// CLASS METHODS
// Returning true if the two values differ for a value less than allowed tolerance.
bool Tolerance::isInTolerance(const double v1, const double v2, const double tolerance) {
    if (std::abs(v1 - v2) <= tolerance) {
        return true;
    }

    return false;
}
