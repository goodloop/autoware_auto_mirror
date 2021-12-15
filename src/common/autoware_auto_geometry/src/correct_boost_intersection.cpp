#include <boost/tuple/tuple.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/foreach.hpp>
#include <iostream>

typedef boost::geometry::model::polygon<boost::tuple<int, int> > Polygon;

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

void test_boost_intersection() {
    Polygon green, blue;
    boost::geometry::read_wkt("POLYGON((0 0,10 0,0 10,10 10,0 0))", green);
    boost::geometry::read_wkt("POLYGON((5 5,6 5 ,5 7,7 8,5 5))", blue);
//    boost::geometry::correct(green);
//    boost::geometry::correct(blue);
    std::deque<Polygon> output;
    boost::geometry::intersection(green, blue, output);
    BOOST_FOREACH(Polygon const& p, output)
                {
                    std::cout << boost::geometry::dsv(p) << std::endl;
                }
}

int main()
{
    test_boost_intersection();
}