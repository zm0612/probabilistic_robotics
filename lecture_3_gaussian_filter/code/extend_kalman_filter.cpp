//
// Created by meng on 2021/10/20.
//

#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <random>

namespace plt = matplotlibcpp;

void GetSimData(const double *init_state,
                const double *measure_noises_cov,
                double delta_t, double duration,
                std::vector<Eigen::Vector4d> &gt,
                std::vector<Eigen::Vector2d> &measures) {
    gt.clear();
    measures.clear();

    const double g = 9.81;
    const double kx = 0.01;
    const double ky = 0.05;
    double x, y, vx, vy;
    x = init_state[0];
    vx = init_state[1];
    y = init_state[2];
    vy = init_state[3];
    Eigen::Vector2d measure;
    auto N = static_cast<unsigned int>(duration / delta_t);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> r_noise(0, measure_noises_cov[0]);
    std::normal_distribution<double> theta_noise(0, measure_noises_cov[1]);

    for (unsigned int i = 0; i < N; ++i) {
        x = x + vx * delta_t;
        vx = vx - kx * vx * vx * delta_t;
        y = y + vy * delta_t;
        vy = vy + (ky * vy * vy - g) * delta_t;

        gt.emplace_back(Eigen::Vector4d(x, vx, y, vy));

        measure[0] = std::sqrt(std::pow(x, 2) + std::pow(y, 2)) + r_noise(gen);
        measure[1] = std::atan2(x, y) + theta_noise(gen);
        measures.emplace_back(measure);
    }
}

void ExtendKalmanFilter() {
    const double init_state[4] = {0.0, 50, 500.0, 0};
    const double measure_noises_variance[2] = {0.5, 0.0001};
    const double delta_t = 0.1;
    const double duration = 50.0;

    std::vector<Eigen::Vector4d> gt;
    std::vector<Eigen::Vector2d> measures;

    Eigen::Vector4d model_noise = {0.1, 0.5, 0.5, 0.5};
    Eigen::Vector2d measure_noise = {measure_noises_variance[0], measure_noises_variance[1]};

    GetSimData(init_state, measure_noises_variance, delta_t, duration, gt, measures);

    const double g = 9.81;
    const double kx = 0.01;
    const double ky = 0.05;
    double x = 0.0, y = 480, vx = 0.0, vy = 0.0;
    Eigen::Vector4d X = {0, 0, 0, 0};

    Eigen::MatrixXd R = model_noise.asDiagonal();
    Eigen::MatrixXd Q = measure_noise.asDiagonal();
    Eigen::Matrix4d Sigma = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d G;
    Eigen::MatrixXd K;
    Eigen::Matrix<double, 2, 4> H;

    std::vector<double> filter_x;
    std::vector<double> filter_y;
    std::vector<double> gt_x;
    std::vector<double> gt_y;

    for (unsigned int i = 0; i < gt.size(); ++i) {
        x = x + vx * delta_t;
        vx = vx - kx * vx * vx * delta_t;
        y = y + vy * delta_t;
        vy = vy + (ky * vy * vy - g) * delta_t;

        X << x, vx, y, vy;

        G << 1, delta_t, 0, 0,
                0, 1 - 2 * kx * vx * delta_t, 0, 0,
                0, 0, 1, delta_t,
                0, 0, 0, 1 + 2 * ky * vy * delta_t;

        double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        H << x / r, 0, y / r, 0,
                (1 / y) / (1 + std::pow(x / y, 2)), 0,
                (-x / y * y) / (1 + std::pow(x / y, 2)), 0;

        Sigma = G * Sigma * G.transpose() + R;
        K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();

        Eigen::Vector2d h;
        h[0] = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        h[1] = std::atan2(x, y);
        X = X + K * (measures[i] - h);

        Sigma = (Eigen::Matrix4d::Identity() - K * H) * Sigma;
        x = X[0];
        vx = X[1];
        y = X[2];
        vy = X[3];

        filter_x.emplace_back(x);
        filter_y.emplace_back(y);
        gt_x.emplace_back(gt[i][0]);
        gt_y.emplace_back(gt[i][2]);
    }

    plt::plot(filter_x, filter_y, "r");
    plt::plot(gt_x, gt_y, "g");
    plt::show();
}

int main() {
    ExtendKalmanFilter();

    return 0;
}