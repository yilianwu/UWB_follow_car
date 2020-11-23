#ifndef KALMAN_HPP
#define KALMAN_HPP

class Kalman_Filter {
public:
    Kalman_Filter();
    Kalman_Filter(double Q, double R, double I);
    double measure(double Z);

private:
    double Q;   // 系統共變異數
    double R;   // 測量雜訊值
    double I;

    double X;   // 上次狀態值
    double P;   // 上次狀態雜訊值
    double Kg;  // 增益值
};

#endif
