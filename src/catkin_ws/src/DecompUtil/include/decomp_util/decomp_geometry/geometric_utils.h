/**
 * @file geometric_utils.h
 * @brief basic geometry utils
 */
#ifndef DECOMP_GEOMETRIC_UTILS_H
#define DECOMP_GEOMETRIC_UTILS_H

#include <Eigen/Eigenvalues>
#include <decomp_util/decomp_basis/data_utils.h>
#include <decomp_util/decomp_geometry/polyhedron.h>
#include <iostream>

/// Calculate eigen values
template <int Dim> Vecf<Dim> eigen_value(const Matf<Dim, Dim> &A) {
  Eigen::SelfAdjointEigenSolver<Matf<Dim, Dim>> es(A);
  return es.eigenvalues();
}

/// Calculate rotation matrix from a vector (aligned with x-axis)
Mat2f vec2_to_rotation(const Vec2f &v);

Mat3f vec3_to_rotation(const Vec3f &v);

/// Sort plannar points in the counter-clockwise order
vec_Vec2f sort_pts(const vec_Vec2f &pts);

/// Find intersection between two lines on the same plane, return false if they
/// are not intersected
bool line_intersect(const std::pair<Vec2f, Vec2f> &v1,
                           const std::pair<Vec2f, Vec2f> &v2, Vec2f &pi);

/// Find intersection between multiple lines
vec_Vec2f line_intersects(const vec_E<std::pair<Vec2f, Vec2f>> &lines);

/// Find extreme points of Polyhedron2D
vec_Vec2f cal_vertices(const Polyhedron2D &poly) ;

// Find extreme points of LinearConstraint2D (Ax <= b)
vec_Vec2f cal_vertices(const LinearConstraint2D &con, const int n_relevent_constraints);

/// Find extreme points of Polyhedron3D
vec_E<vec_Vec3f> cal_vertices(const Polyhedron3D &poly);

/// Get the convex hull of a 2D points array, use wrapping method
vec_Vec2f cal_convex_hull(const vec_Vec2f &pts);

Polyhedron2D get_convex_hull(const vec_Vec2f &pts);

/// Minkowski sum, add B to A with center Bc
Polyhedron2D minkowski_sum(const Polyhedron2D &A, const Polyhedron2D &B,
                                  const Vec2f &Bc);

#endif
