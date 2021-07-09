#include "kalman.hpp"

Kalman_Filter::Kalman_Filter()
    : Kalman_Filter(5, 3, 1)
{}

Kalman_Filter::Kalman_Filter(double Q, double R, double I)
    : Q(Q), R(R), I(I), X(0), P(0), Kg(0)
{}

double Kalman_Filter::measure(double Z)
{
    // Predict
    double Xk = this->X;
    double Pk = this->P + this->Q;
    // Update
    this->X = Xk + this->Kg * (Z - Xk);
    this->Kg = Pk / (Pk + this->R);
    this->P = (this->I - this->Kg) * Pk;

    return this->X;
}
