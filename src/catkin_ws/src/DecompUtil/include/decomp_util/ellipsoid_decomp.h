/**
 * @file ellipsoid_decomp.h
 * @brief EllipsoidDecomp Class
 */
#ifndef ELLIPSOID_DECOMP_H
#define ELLIPSOID_DECOMP_H

#include <memory>
#include <thread>
#include <decomp_util/line_segment.h>
#include <mpc_tools/instrumentation_timer.h>

/**
 * @brief EllipsoidDecomp Class
 *
 * EllipsoidDecomp takes input as a given path and find the Safe Flight Corridor around it using Ellipsoids
 */
template <int Dim>
class EllipsoidDecomp {
public:
 ///Simple constructor
 
  EllipsoidDecomp()
  {}

 /**
  * @brief Basic constructor
  * @param origin The origin of the global bounding box
  * @param dim The dimension of the global bounding box
  */
 EllipsoidDecomp(const Vecf<Dim> &origin, const Vecf<Dim> &dim) {
   global_bbox_min_ = origin;
   global_bbox_max_ = origin + dim;
 }

 ///Set obstacle points
 void set_obs(const vec_Vecf<Dim> &obs) { obs_ = obs; }

 ///Set dimension of bounding box
 void set_local_bbox(const Vecf<Dim>& bbox) { local_bbox_ = bbox; }

 /**
  * @brief Tighten polyhedron i with a specified distance seen from the perspective of a point inside the polyhedron
  * 
  * @param i Polyhedron index
  * @param pt_inside Point inside the polyhedron
  * @param distance Distance to tighten the polyhedron with
  */
 void tighten_polyhedron(const size_t i, const Vecf<Dim> pt_inside, const decimal_t distance)
 {
  for (unsigned int j = 0; j < polyhedrons_[i].vs_.size(); j++)
  {
    // Determine direction of normal vector and sign of constant in linear constraint (similar to LinearConstraint constructor)
    auto n = polyhedrons_[i].vs_[j].n_;
    decimal_t c = polyhedrons_[i].vs_[j].p_.dot(n);
    if (n.dot(pt_inside) - c > 0) {
      n = -n;
    }
    // Normalize normal vector
    n = n.normalized();

    // Tighten constraint by specified distance
    polyhedrons_[i].vs_[j].p_ = polyhedrons_[i].vs_[j].p_ - distance*n;
  }
 }

 /**
  * @brief Store the linear constraints corresponding to the calculated polyhedrons in an external vector
  * 
  * @param poly_constraints External linear constraints vector
  * @param distance Distance to tighten the polyhedron constraints with
  */
 void set_constraints(std::vector<LinearConstraint<Dim>>& poly_constraints, const decimal_t distance = 0)
 {
  // Remove all previously stored constraints
  poly_constraints.clear();

  // Calculate and store new constraints
  size_t idx_path = 0;
  for (size_t i = 0; i < polyhedrons_.size(); i++)
  {
    // Create a point inside the polyhedron: on the corresponding line segment
    const Vecf<Dim> pt_inside = (path_[idx_path] + path_[idx_path+1])/2;

    // Store polyhedron constraints
    poly_constraints.emplace_back(LinearConstraint<Dim>(pt_inside, polyhedrons_[i].hyperplanes(), distance));

    // Tighten the constraints (to align with linear constraints)
    if (distance > 0)
    {
      tighten_polyhedron(i, pt_inside, distance);
    }

    // Update idx_path (extra increase in case of circular elements)
    if (is_path_circle_only_) idx_path++;
    idx_path++;
  }
 }

 ///Get the path that is used for dilation
 vec_Vecf<Dim> get_path() const { return path_; }

 ///Get the Safe Flight Corridor
 vec_E<Polyhedron<Dim>> get_polyhedrons() const { return polyhedrons_; }

 ///Get the ellipsoids
 vec_E<Ellipsoid<Dim>> get_ellipsoids() const { return ellipsoids_; }

 ///Get the constraints of SFC as \f$Ax\leq b \f$
 vec_E<LinearConstraint<Dim>> get_constraints() const {
   vec_E<LinearConstraint<Dim>> constraints;
   constraints.resize(polyhedrons_.size());
   for (unsigned int i = 0; i < polyhedrons_.size(); i++){
     const Vecf<Dim> pt = (path_[i] + path_[i+1])/2;
     constraints[i] = LinearConstraint<Dim>(pt, polyhedrons_[i].hyperplanes());
   }
   return constraints;
 }

/**
* @brief Decomposition thread
* @param path The path to dilate
* @param obs_path_points The obstacle points for each path segment
* @param offset_x Offset added to the long semi-axis, default is 0
* @param is_path_circle Indicator that path only consists of sets of two path elements giving line segments, so line segments are not constructed between every path segment
*/
#ifndef DECOMP_OLD
void dilate(const vec_Vecf<Dim> &path, const std::vector<std::unique_ptr<vec_Vecf<Dim>>> &obs_path_points, double offset_x = 0, bool is_path_circle_only = false) {
  PROFILE_FUNCTION();
    
  is_path_circle_only_ = is_path_circle_only;
  const unsigned int n_path = path.size();
  unsigned int n_segments = n_path-1;
  if (is_path_circle_only_) n_segments = n_path/2;

  lines_.resize(n_segments);
  ellipsoids_.resize(n_segments);
  polyhedrons_.resize(n_segments);

  // Create line segments and corresponding ellipsoids and polyhedrons based on a path with ellipsoidal and circular elements
  unsigned int idx_path = 0;
  for (unsigned int i = 0; i < n_segments; i++)  {
    lines_[i] = std::make_shared<LineSegment<Dim>>(path[idx_path], path[idx_path+1]);
    lines_[i]->set_local_bbox(local_bbox_);
    lines_[i]->set_obs_ptr(obs_path_points[i].get());
    lines_[i]->dilate(offset_x);
    ellipsoids_[i] = lines_[i]->get_ellipsoid();
    polyhedrons_[i] = lines_[i]->get_polyhedron();

    if (is_path_circle_only_) idx_path++;
    idx_path++;
  }

  path_ = path;

  if (global_bbox_min_.norm() != 0 || global_bbox_max_.norm() != 0) {
    for(auto& it: polyhedrons_)
      add_global_bbox(it);
  }
}
#endif
#ifdef DECOMP_OLD
void dilate(const vec_Vecf<Dim> &path, double offset_x = 0, bool is_path_circle_only = false) {
  PROFILE_FUNCTION();
    
  is_path_circle_only_ = is_path_circle_only;
  const unsigned int n_path = path.size();
  unsigned int n_segments = n_path-1;
  if (is_path_circle_only_) n_segments = n_path/2;

  lines_.resize(n_segments);
  ellipsoids_.resize(n_segments);
  polyhedrons_.resize(n_segments);

  // Create line segments and corresponding ellipsoids and polyhedrons based on a path with ellipsoidal and circular elements
  unsigned int idx_path = 0;
  for (unsigned int i = 0; i < n_segments; i++)  {
    lines_[i] = std::make_shared<LineSegment<Dim>>(path[idx_path], path[idx_path+1]);
    lines_[i]->set_local_bbox(local_bbox_);
    lines_[i]->set_obs(obs_);
    lines_[i]->dilate(offset_x);
    ellipsoids_[i] = lines_[i]->get_ellipsoid();
    polyhedrons_[i] = lines_[i]->get_polyhedron();

    if (is_path_circle_only_) idx_path++;
    idx_path++;
  }

  path_ = path;

  if (global_bbox_min_.norm() != 0 || global_bbox_max_.norm() != 0) {
    for(auto& it: polyhedrons_)
      add_global_bbox(it);
  }
}
#endif

void calculatePolyhedron(const Vecf<Dim> &local_bbox, const vec_Vecf<Dim> &obs, const vec_Vecf<Dim> &path, const int idx_path, const unsigned int index, const double offset_x = 0)
{
  std::shared_ptr<LineSegment<Dim>> lines = std::make_shared<LineSegment<Dim>>(path[idx_path], path[idx_path+1]);
  {
    PROFILE_SCOPE("set vars line");
    lines->set_local_bbox(local_bbox);
    lines->set_obs(obs);
  }
  {
    PROFILE_SCOPE("dilate line");
    lines->dilate(offset_x);
  }
  lines_[index] = lines;
}


protected:
 template<int U = Dim>
   typename std::enable_if<U == 2>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** add bound along X, Y axis

     //*** X
     Vs.add(Hyperplane2D(Vec2f(global_bbox_max_(0), 0), Vec2f(1, 0)));
     Vs.add(Hyperplane2D(Vec2f(global_bbox_min_(0), 0), Vec2f(-1, 0)));
     //*** Y
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_max_(1)), Vec2f(0, 1)));
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_min_(1)), Vec2f(0, -1)));
   }

 template<int U = Dim>
   typename std::enable_if<U == 3>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** add bound along X, Y, Z axis
     //*** Z
     Vs.add(Hyperplane3D(Vec3f(0, 0, global_bbox_max_(2)), Vec3f(0, 0, 1)));
     Vs.add(Hyperplane3D(Vec3f(0, 0, global_bbox_min_(2)), Vec3f(0, 0, -1)));

     //*** X
     Vs.add(Hyperplane3D(Vec3f(global_bbox_max_(0), 0, 0), Vec3f(1, 0, 0)));
     Vs.add(Hyperplane3D(Vec3f(global_bbox_min_(0), 0, 0), Vec3f(-1, 0, 0)));
     //*** Y
     Vs.add(Hyperplane3D(Vec3f(0, global_bbox_max_(1), 0), Vec3f(0, 1, 0)));
     Vs.add(Hyperplane3D(Vec3f(0, global_bbox_max_(1), 0), Vec3f(0, -1, 0)));
   }

 vec_Vecf<Dim> path_;
 bool is_path_circle_only_;
 vec_Vecf<Dim> obs_;

 vec_E<Ellipsoid<Dim>> ellipsoids_;
 vec_E<Polyhedron<Dim>> polyhedrons_;
 std::vector<std::shared_ptr<LineSegment<Dim>>> lines_;

 Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()};
 Vecf<Dim> global_bbox_min_{Vecf<Dim>::Zero()}; // bounding box params
 Vecf<Dim> global_bbox_max_{Vecf<Dim>::Zero()};

};

typedef EllipsoidDecomp<2> EllipsoidDecomp2D;

typedef EllipsoidDecomp<3> EllipsoidDecomp3D;
#endif
