

#include <decomp_util/decomp_geometry/polyhedron.h>

///Hyperplane class
template Hyperplane<2>::Hyperplane();
template Hyperplane<2>::Hyperplane(const Vecf<2>& p, const Vecf<2>& n);
template Hyperplane<3>::Hyperplane();
template Hyperplane<3>::Hyperplane(const Vecf<3>& p, const Vecf<3>& n);


///Polyhedron class
template Polyhedron<2>::Polyhedron();
template Polyhedron<2>::Polyhedron(const vec_E<Hyperplane<2>>& vs);
template Polyhedron<3>::Polyhedron();
template Polyhedron<3>::Polyhedron(const vec_E<Hyperplane<3>>& vs);



///[A, b] for \f$Ax < b\f$
/// Linear constraint class
template LinearConstraint<2>::LinearConstraint();
template LinearConstraint<2>::LinearConstraint(const MatDNf<2>& A, const VecDf& b);
template LinearConstraint<2>::LinearConstraint(const Vecf<2> p0, const vec_E<Hyperplane<2>>& vs, const decimal_t distance);
template LinearConstraint<3>::LinearConstraint();
template LinearConstraint<3>::LinearConstraint(const MatDNf<3>& A, const VecDf& b);
template LinearConstraint<3>::LinearConstraint(const Vecf<3> p0, const vec_E<Hyperplane<3>>& vs, const decimal_t distance);



