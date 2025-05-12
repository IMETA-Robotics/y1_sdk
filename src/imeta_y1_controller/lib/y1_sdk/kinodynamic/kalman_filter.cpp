#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(double dt, double process_noise, double measurement_noise)
        : dt_(dt) {
        // 状态转移矩阵
        A_ = Eigen::Matrix2d::Identity();
        A_(0, 1) = dt_;

        // 控制输入矩阵
        B_ = Eigen::Vector2d(0.5 * dt_ * dt_, dt_);

        // 观测矩阵
        H_ = Eigen::RowVector2d(1, 0);

        // 过程噪声协方差
        Q_ = Eigen::Matrix2d::Identity() * process_noise;

        // 观测噪声协方差
        R_ = Eigen::Matrix<double, 1, 1>::Constant(measurement_noise);

        // 初始估计协方差
        P_ = Eigen::Matrix2d::Identity();

        // 初始状态
        x_ = Eigen::Vector2d::Zero();
    }

    void predict(double u) {
        x_ = A_ * x_ + B_ * u;
        P_ = A_ * P_ * A_.transpose() + Q_;
    }

    void update(double z) {
        Eigen::Matrix<double, 1, 1> y;
        y << z - H_ * x_;
        Eigen::Matrix<double, 1, 1> S = H_ * P_ * H_.transpose() + R_;
        Eigen::Vector2d K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y(0, 0);
        P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
    }

    double getPosition() const { return x_(0); }
    double getVelocity() const { return x_(1); }

private:
    double dt_;
    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;
    Eigen::RowVector2d H_;
    Eigen::Matrix2d Q_;
    Eigen::Matrix<double, 1, 1> R_;
    Eigen::Matrix2d P_;
    Eigen::Vector2d x_;
};
