#pragma once

//#define BOOST_NO_CXX11_VARIADIC_TEMPLATES this cause error in VS2015
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#undef min
#undef max
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel; // EPIC


using Point_2 = Kernel::Point_2;
using Vector_2 = Kernel::Vector_2;

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;

using Segment_2 = CGAL::Segment_2<Kernel>;
using Ray_2 = CGAL::Ray_2<Kernel>;
using Line_2 = CGAL::Line_2<Kernel>;
