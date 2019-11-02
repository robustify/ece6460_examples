#pragma once

#include <geometry_msgs/Point.h>
#include <ece6460_mono_example/BezierCurve.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

using namespace Eigen;

namespace ece6460_mono_example {

static const unsigned int FACTORIAL_TABLE_[] = {
  1,
  1,
  2,
  6,
  24,
  120,
  720,
  5040,
  40320,
  362880,
  3628800
};

typedef std::vector<geometry_msgs::Point> BezierVisualization;

class Bezier{

public:
  Bezier();

  static double fitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data, BezierCurve& bezier_curve, int order)
  {
    bezier_curve.points.resize(order + 1);
    size_t N = data->points.size();

    // Not enough points for a good fit
    if (N < order) {
      bezier_curve.points.clear();
      return false;
    }

    MatrixXd tau_mat;
    std::vector<double> tau_map;
    constructTauMatrix(data, order, N, tau_mat, tau_map);

    // Construct regression matrix
    MatrixXd regression_mat;
    regression_mat.setZero(2, N);
    for (unsigned int i = 0; i < N; i++) {
      regression_mat(0, i) = data->points[i].x;
      regression_mat(1, i) = data->points[i].y;
    }

    // Perform least squares fit
    MatrixXd result_mat = regression_mat * tau_mat;

    // Populate output
    for (int i = 0; i <= order; i++) {
      bezier_curve.points[i].x = result_mat(0, i);
      bezier_curve.points[i].y = result_mat(1, i);
    }

    return true;
  }

  static void constructTauMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data, int order, int num_points, MatrixXd& tau_mat, std::vector<double>& tau_map)
  {
    // Construct tau matrix
    tau_map.resize(num_points, 0.0);
    double max_dist = 0.0;
    double min_dist = INFINITY;
    std::vector<double> d(num_points, 0.0);
    for (size_t i = 1; i < num_points; i++) {
      const pcl::PointXYZ& p = data->points[i];
      d[i] = sqrt(p.x * p.x + p.y * p.y);
      if (d[i] > max_dist) {
        max_dist = d[i];
      }
      if (d[i] < min_dist) {
        min_dist = d[i];
      }
    }
    for (size_t i = 0; i < num_points; i++) {
      tau_map[i] = (d[i] - min_dist) / (max_dist - min_dist);
    }

    tau_mat.setZero(order + 1, num_points);
    for (int i = 0; i <= order; i++) {
      for (size_t j = 0; j < num_points; j++) {
        int coeff = FACTORIAL_TABLE_[order] / FACTORIAL_TABLE_[i] / FACTORIAL_TABLE_[order - i];
        double t1 = 1.0;
        double t2 = 1.0;
        for (int k = 0; k < order - i; k++) {
          t1 *= (1 - tau_map[j]);
        }
        for (int k = 0; k < i; k++) {
          t2 *= tau_map[j];
        }
        tau_mat(i, j) = coeff * t1 * t2;
      }
    }

    // Invert tau matrix -- SVD method used since Moore-Penrose requires inverting a square matrix with zero determinant
    JacobiSVD<MatrixXd> svd(tau_mat, ComputeThinU | ComputeThinV);
    MatrixXd u = svd.matrixU();
    MatrixXd v = svd.matrixV();
    VectorXd svals = svd.singularValues();
    tau_mat = v * svals.asDiagonal().inverse() * u.transpose();
  }

  static void bezierPoint(const BezierCurve& segment, double tau, geometry_msgs::Point& point)
  {
    int order = segment.points.size() - 1;
    point.x = 0;
    point.y = 0;
    for (unsigned int i = 0; i < segment.points.size(); i++) {
      int coeff = FACTORIAL_TABLE_[order] / FACTORIAL_TABLE_[i] / FACTORIAL_TABLE_[order - i];
      double t1 = 1.0;
      double t2 = 1.0;
      for (int k = 0; k < order - i; k++) {
        t1 *= (1 - tau);
      }
      for (int k = 0; k < i; k++) {
        t2 *= tau;
      }
      point.x += coeff * t1 * t2 * segment.points[i].x;
      point.y += coeff * t1 * t2 * segment.points[i].y;
    }
  }

  static void visualize(const BezierCurve& bezier_segment, BezierVisualization& points)
  {
    points.clear();
    geometry_msgs::Point tpoint;

    for (double tau = 0.0; tau <= 1.0; tau += 0.02) {
      bezierPoint(bezier_segment, tau, tpoint);
      points.push_back(tpoint);
    }
  }

  static void visualize(const BezierCurve& bezier_segment, nav_msgs::Path& path_msg)
  {
    path_msg.poses.clear();
    geometry_msgs::PoseStamped tpose;

    for (double tau = 0.0; tau <= 1.0; tau += 0.02) {
      geometry_msgs::Point tpoint;
      bezierPoint(bezier_segment, tau, tpoint);
      tpose.pose.position = tpoint;
      tpose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(tpose);
    }
  }
};

}
