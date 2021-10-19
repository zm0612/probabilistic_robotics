#include <vector>
#include <random>
#include <iostream>
#include <Eigen/Dense>

std::vector<double> GetSimData(double init_p, double init_v, double init_a,
                               double duration, double noise, double period,
                               std::vector<double> &ground_truth) {
    std::vector<double> positions;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> normal_distribution(0, noise);

    auto N = static_cast<unsigned int > (duration / period);

    double p_k;
    for (unsigned int i = 0; i < N; ++i) {
        p_k = init_p + init_v * (i * period) + 0.5 * init_a * std::pow(period, 2.0);
        ground_truth.emplace_back(p_k);
        p_k = p_k + normal_distribution(gen);
        positions.emplace_back(p_k);
    }

    return positions;
}

void KalmanFilter() {
    double init_p = 3.5;
    double init_v = 1.3;
    double init_a = 0.05;
    double measure_noise = 0.2;
    double duration = 100;
    double T = 1.0;
    std::vector<double> ground_truth;
    std::vector<double> measures = GetSimData(init_p, init_v, init_a, duration,
                                              measure_noise, T, ground_truth);

    Eigen::Vector2d state(0, 0);
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::RowVector2d C;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = 0.0001;
    R(1, 1) = 0.0001;
    Eigen::Matrix2d Sigma = Eigen::Matrix2d::Zero();
    Eigen::MatrixXd K;
    Eigen::Matrix<double, 1, 1> Q(0.04);

    A << 1, T, 0, 1;
    B << 0.5 * T * T, T;
    C << 1, 0;

    auto N = static_cast<unsigned int >(duration / T);
    for (unsigned int i = 0; i < N; ++i) {
        state = A * state + B * init_a;
        Sigma = A * Sigma * A.transpose() + R;

        K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Q).inverse();
        state = state + K * (measures[i] - C * state);
        Sigma = (Eigen::Matrix2d::Identity() - K * C) * Sigma;
        std::cout << "iter: " << i << " state: " << state.transpose() << " | gt(p): " << ground_truth[i] << std::endl;
    }
}

int main() {
    KalmanFilter();

    return 0;
}
