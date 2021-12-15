#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
namespace bgm = bg::model;
typedef bgm::polygon<bgm::d2::point_xy<double> > polygon;

int main() {
    polygon poly1, poly2;
    bg::read<bg::format_wkt>(poly1, "POLYGON((0 0,10 0,0 10,10 10,0 0))");
    bg::read<bg::format_wkt>(poly2, "POLYGON((5 5,6 5,5 7,7 8,5 5))");

    std::cout << bg::wkt(poly1) << "\n";
    std::cout << bg::wkt(poly2) << "\n";
    std::deque<polygon> output;
    bg::intersection(poly1, poly2, output);


    double area = 0;
    for (auto& p : output)
        std::cout <<  bg::dsv(p);
}