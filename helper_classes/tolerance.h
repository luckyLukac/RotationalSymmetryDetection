#ifndef TOLERANCE_H
#define TOLERANCE_H


namespace Symmetry {
    namespace Difference {
        double difference(const double v1, const double v2);
    };

    namespace Tolerance {
        bool isInTolerance(const double v1, const double v2, const double tolerance);  // Returning true if the two values differ for a value less than allowed tolerance.
    };
};

#endif // TOLERANCE_H
