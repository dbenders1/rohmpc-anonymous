
#include <decomp_util/decomp_geometry/ellipsoid.h>


template Ellipsoid<2>::Ellipsoid();
template Ellipsoid<2>::Ellipsoid(const Matf<2, 2>& C, const Vecf<2>& d);

template Ellipsoid<3>::Ellipsoid();
template Ellipsoid<3>::Ellipsoid(const Matf<3, 3>& C, const Vecf<3>& d);
