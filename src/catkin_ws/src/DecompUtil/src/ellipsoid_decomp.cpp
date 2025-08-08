
#include <decomp_util/ellipsoid_decomp.h>

#ifndef THREADING
template EllipsoidDecomp<2>::EllipsoidDecomp();
template EllipsoidDecomp<3>::EllipsoidDecomp();
#endif
#ifdef THREADING
template EllipsoidDecomp<2>::EllipsoidDecomp(const int n_threads);
template EllipsoidDecomp<3>::EllipsoidDecomp(const int n_threads);
#endif

template EllipsoidDecomp<2>::EllipsoidDecomp(const Vecf<2> &origin, const Vecf<2> &dim);
template EllipsoidDecomp<3>::EllipsoidDecomp(const Vecf<3> &origin, const Vecf<3> &dim);