#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "polycal/cusum.hpp"
#include "polycal/ekf.hpp"

namespace py = pybind11;
using namespace polycal;

namespace {

Sophus::SE3d matrix_to_se3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);
    Eigen::Vector3d translation = T.block<3, 1>(0, 3);
    return Sophus::SE3d(rotation, translation);
}

}  // namespace

PYBIND11_MODULE(_polycal_cpp, m) {
    m.doc() = "polycal C++ backend";

    py::class_<ExtrinsicEKF::UpdateResult>(m, "UpdateResult")
        .def_readonly("innovation", &ExtrinsicEKF::UpdateResult::innovation)
        .def_readonly("innovation_cov", &ExtrinsicEKF::UpdateResult::innovation_cov);

    py::class_<ExtrinsicEKF>(m, "ExtrinsicEKF")
        .def(py::init([](Eigen::Matrix4d T_lc_init,
                         Mat6d P_init,
                         Mat6d Q) {
                return ExtrinsicEKF(matrix_to_se3(T_lc_init), P_init, Q);
            }),
            py::arg("T_lc_init"),
            py::arg("P_init"),
            py::arg("Q"))
        .def("predict", &ExtrinsicEKF::predict, py::arg("dt"))
        .def("update", [](ExtrinsicEKF& self,
                          Eigen::Matrix4d T_cam,
                          Eigen::Matrix4d T_lidar,
                          Mat6d R) {
                self.update(matrix_to_se3(T_cam), matrix_to_se3(T_lidar), R);
            },
            py::arg("T_cam_odom"),
            py::arg("T_lidar_odom"),
            py::arg("R"))
        .def("update_with_cusum", [](ExtrinsicEKF& self,
                                     Eigen::Matrix4d T_cam,
                                     Eigen::Matrix4d T_lidar,
                                     Mat6d R) {
                return self.update_with_cusum(
                    matrix_to_se3(T_cam), matrix_to_se3(T_lidar), R);
            },
            py::arg("T_cam_odom"),
            py::arg("T_lidar_odom"),
            py::arg("R"))
        .def("compute_residual", [](ExtrinsicEKF& self,
                                    Eigen::Matrix4d T_cam,
                                    Eigen::Matrix4d T_lidar) {
                return self.compute_residual(
                    matrix_to_se3(T_cam), matrix_to_se3(T_lidar));
            },
            py::arg("T_cam_odom"),
            py::arg("T_lidar_odom"))
        .def("T_lc", [](const ExtrinsicEKF& self) {
                return self.T_lc().matrix();
            })
        .def("P", &ExtrinsicEKF::P)
        .def("x", &ExtrinsicEKF::x);

    py::class_<CUSUMConfig>(m, "CUSUMConfig")
        .def(py::init<>())
        .def_readwrite("kappa", &CUSUMConfig::kappa)
        .def_readwrite("threshold", &CUSUMConfig::threshold)
        .def_readwrite("dof", &CUSUMConfig::dof);

    py::class_<CUSUMCalibration>(m, "CUSUMCalibration")
        .def_readonly("kappa", &CUSUMCalibration::kappa)
        .def_readonly("mean", &CUSUMCalibration::mean)
        .def_readonly("std_dev", &CUSUMCalibration::std_dev)
        .def_readonly("percentile", &CUSUMCalibration::percentile)
        .def_readonly("n", &CUSUMCalibration::n);

    py::class_<CUSUMDetector>(m, "CUSUMDetector")
        .def(py::init<const CUSUMConfig&>(),
             py::arg("config") = CUSUMConfig{})
        .def("update", [](CUSUMDetector& self,
                          Vec6d innovation,
                          Mat6d innovation_cov) {
                return self.update(innovation, innovation_cov);
            },
            py::arg("innovation"),
            py::arg("innovation_cov"))
        .def("reset", &CUSUMDetector::reset)
        .def("statistic", &CUSUMDetector::statistic)
        .def("steps_to_alarm", &CUSUMDetector::steps_to_alarm)
        .def_static("compute_innovation_cov",
                    &CUSUMDetector::compute_innovation_cov,
                    py::arg("H"), py::arg("P"), py::arg("R"))
        .def_static("calibrate_kappa",
                    &CUSUMDetector::calibrate_kappa,
                    py::arg("innovations"),
                    py::arg("covs"),
                    py::arg("target_percentile") = 0.80);
}
