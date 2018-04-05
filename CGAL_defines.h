//
// Created by t-idkess on 18-Mar-18.
//

#ifndef INC_2_3_CGAL_DEFINES_H
#define INC_2_3_CGAL_DEFINES_H

#include <CGAL/Cartesian.h>
#include <CGAL/Gmpq.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Triangulation_2_traits_3.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <list>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename Kernel::Triangle_2 Triangle_2;
typedef typename Kernel::Vector_2 Vector_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef typename CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Triangulation_vertex_base_2<Kernel>                      Vb;
typedef CGAL::Triangulation_face_base_with_info_2<std::string,Kernel>    Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef typename CGAL::Constrained_triangulation_2<Kernel,TDS,Itag> ConstrainedTriangulation;


#endif //INC_2_3_CGAL_DEFINES_H
