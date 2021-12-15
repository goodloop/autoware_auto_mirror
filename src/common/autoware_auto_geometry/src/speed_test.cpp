#include <iostream>
#include <chrono>
#include <vector>
#include <iostream>
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>

namespace bg = boost::geometry;

struct MyPoint {
    double x, y;
};

typedef std::vector<MyPoint> MyPolygon;

BOOST_GEOMETRY_REGISTER_POINT_2D(MyPoint, double, bg::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_LINESTRING(MyPolygon)

template < typename Polygon, typename Point >
std::deque<Polygon> test_polygon() {

    Polygon poly1 { {0.0,  0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0,  0.0}, {0.05,  0.0}, };
    Polygon poly2 { {0.5, -0.5}, {0.5, 0.5}, {1.5, 0.5}, {1.5, -0.5},  {0.5, -0.5}, };

    std::deque<Polygon> output;
    boost::geometry::intersection(poly1, poly2, output);

    return output;
}

int main()
{

    time_t begin,end; // time_t is a datatype to store time values.

    time (&begin); // note time before execution

    for (auto& p : test_polygon< MyPolygon, MyPoint >())
        std::cout <<  bg::dsv(p);


//    time (&end); // note time after execution
//
//    double difference = difftime (end,begin);
//    printf ("time taken for function() %.2lf seconds.\n", difference );

    return 0;
}

