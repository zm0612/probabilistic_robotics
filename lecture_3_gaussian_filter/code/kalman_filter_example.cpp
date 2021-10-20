#include "matplotlibcpp.h"

#include <vector>
#include <random>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace plt = matplotlibcpp;

void AnalyzeObservability(const Eigen::MatrixXd Q) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q);
    std::cout << "eigen value: " << svd.singularValues().eval().transpose() << std::endl;
}

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
        p_k = init_p + init_v * (i * period) + 0.5 * init_a * std::pow(period * i, 2.0);
        ground_truth.emplace_back(p_k);
        p_k = p_k + normal_distribution(gen);
        positions.emplace_back(p_k);
    }

    return positions;
}

void KalmanFilter() {
    double init_p = 3.5;
    double init_v = 1.3;
    double init_a = 0.1;
    double measure_noise = 10;
    double duration = 60;
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

    std::vector<double> filter_p;
    std::vector<double> filter_v;
    std::vector<double> t;
    std::vector<double> gt_v;
    std::vector<double> gt_p;
    auto N = static_cast<unsigned int >(duration / T);
    for (unsigned int i = 0; i < N; ++i) {
        state = A * state + B * init_a;
        Sigma = A * Sigma * A.transpose() + R;

        K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Q).inverse();
        state = state + K * (measures[i] - C * state);
        Sigma = (Eigen::Matrix2d::Identity() - K * C) * Sigma;
        std::cout << "iter: " << i << " state: " << state.transpose() << " | gt(p): " << ground_truth[i] << std::endl;
        filter_p.emplace_back(state[0]);
        filter_v.emplace_back(state[1]);
        t.emplace_back(i);
        double gt_v_k = init_v + init_a * (i * T);
        gt_v.emplace_back(gt_v_k);
        double gt_p_k = init_p + init_v * (i * T) + 0.5 * init_a * std::pow((i * T), 2);
        gt_p.emplace_back(gt_p_k);
    }

    plt::suptitle("Kalman Filter");
    plt::subplot2grid(2, 1, 0, 0);
    plt::plot(t, filter_p, "r");
    plt::plot(t, measures, "b");
    plt::plot(t, gt_p, "g");
    plt::named_plot("gt_p", t, gt_p);
    plt::named_plot("measure_p", t, measures);
    plt::named_plot("filtered_p", t, filter_p);
    plt::title("position");
    plt::legend();

    plt::subplot2grid(2, 1, 1, 0);
    plt::plot(t, gt_v, "g");
    plt::plot(t, filter_v, "r");
    plt::named_plot("gt_v", t, gt_v, "g");
    plt::named_plot("filtered_v", t, filter_v, "r");
    plt::title("velocity");
    plt::legend();

    plt::show();
}

int main() {
    KalmanFilter();

    return 0;
}
