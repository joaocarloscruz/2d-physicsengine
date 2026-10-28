#pragma once
// Oscillation measurement and the backward-Euler oracle shared by the soft
// joint tests (mouse, weld, wheel).
#include <cmath>
#include <complex>
#include <vector>

namespace JointOscillation {
constexpr double Pi = 3.14159265358979323846;

struct Measured {
    double frequency = 0;    // undamped natural frequency, Hz
    double dampingRatio = 0; // from the logarithmic decrement
    int peaks = 0;
};
// Samples are x(n*h) of a decaying oscillation about zero. Peaks are refined
// with a parabola through the three samples around each local maximum.
inline Measured Measure(const std::vector<double>& x, double h) {
    std::vector<double> times, values;
    for (std::size_t i = 1; i + 1 < x.size(); ++i) {
        if (x[i] > 0 && x[i] >= x[i - 1] && x[i] > x[i + 1]) {
            const double a = x[i - 1], b = x[i], c = x[i + 1];
            const double denominator = a - 2 * b + c;
            const double offset = denominator != 0 ? 0.5 * (a - c) / denominator : 0;
            times.push_back((double(i) + offset) * h);
            values.push_back(b - 0.25 * (a - c) * offset);
        }
    }
    Measured m;
    m.peaks = int(times.size());
    if (times.size() < 3) return m;
    const std::size_t n = times.size() - 1;
    const double period = (times[n] - times[0]) / double(n);
    const double delta = std::log(values[0] / values[n]) / double(n);
    m.dampingRatio = delta / std::sqrt(4 * Pi * Pi + delta * delta);
    m.frequency = 1 / period / std::sqrt(1 - m.dampingRatio * m.dampingRatio);
    return m;
}
// The soft constraint is the backward Euler step of x'' = -w^2 x - 2 z w x'.
// Its discrete eigenvalue is 1/(1 - h*s) for each continuous root s; mapping
// back through log(.)/h gives the frequency and damping a run should show.
inline Measured BackwardEuler(double frequency, double dampingRatio, double h) {
    const double omega = 2 * Pi * frequency;
    const std::complex<double> s(-dampingRatio * omega, omega * std::sqrt(1 - dampingRatio * dampingRatio));
    const std::complex<double> z = 1.0 / (1.0 - h * s);
    const std::complex<double> effective = std::log(z) / h;
    Measured m;
    m.frequency = std::abs(effective) / (2 * Pi);
    m.dampingRatio = -effective.real() / std::abs(effective);
    return m;
}
}
