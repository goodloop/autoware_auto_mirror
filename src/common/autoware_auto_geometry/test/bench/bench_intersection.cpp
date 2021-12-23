#include <geometry/common_2d.hpp>
#include <list>
#include <geometry/intersection.hpp>
#include <benchmark/benchmark.h>


struct TestPoint
{
    autoware::common::types::float32_t x;
    autoware::common::types::float32_t y;
};

using point_type = autoware::common::geometry::bg::model::point<double, 2, autoware::common::geometry::bg::cs::cartesian>;
using polygon_type = autoware::common::geometry::bg::model::ring<point_type>;

static void bench_convex_polygon_intersection_2d(benchmark::State& state) {

    polygon_type polygon1{{ 6, 1},{ 6, 8},{ 2, 8},{-2, 4},{ 1, 1},{ 6, 1}};
    polygon_type polygon2{{1, 2},{2, 6},{4, 2},{1, 2}};

    for (auto _ : state)
        autoware::common::geometry::convex_polygon_intersection2d(polygon1, polygon2);

}


// Register the function as a benchmark

BENCHMARK(bench_convex_polygon_intersection_2d);
















