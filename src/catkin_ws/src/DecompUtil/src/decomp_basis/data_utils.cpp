/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#include <decomp_util/decomp_basis/data_utils.h>

// Prebuild the templated functions

//template vec_E<Vec3f> transform_vec(const vec_E<Vec3f> &t, const Aff3f &tf);
template decimal_t total_distance<Vec3f>(const vec_E<Vec3f>& vs);
template decimal_t total_distance<Vec3i>(const vec_E<Vec3i>& vs);
