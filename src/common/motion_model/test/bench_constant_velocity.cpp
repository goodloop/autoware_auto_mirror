#include <benchmark/benchmark.h>
#include <motion_model/constant_velocity.hpp>
#include <motion_model/constant_velocity_new.hpp>
#include <iostream>
#include <memory>

using autoware::motion::motion_model::MotionModel;
using autoware::motion::motion_model::ConstantVelocity;
using autoware::motion::motion_model::ConstantVelocityNew;
using Eigen::Matrix;
using autoware::common::types::float32_t;


static void ConstantVelocitySingleAllocation(benchmark::State & state)
{
  ConstantVelocity model;
  const uint64_t sz = 4U;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  Matrix<float32_t, sz, sz> F;
  for (auto _ : state) {
    model.compute_jacobian(F, milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}

static void ConstantVelocityMultipleAllocations(benchmark::State & state)
{
  std::unique_ptr<MotionModel<4U>> model = std::make_unique<ConstantVelocity>();
  const uint64_t sz = 4U;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  for (auto _ : state) {
    Matrix<float32_t, sz, sz> F;
    model->compute_jacobian(F, milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}


static void ConstantVelocityNewCopy(benchmark::State & state)
{
  ConstantVelocityNew model;
  const uint64_t sz = 4U;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  Matrix<float32_t, sz, sz> F;
  for (auto _ : state) {
    F = model.compute_jacobian(milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}

static void ConstantVelocityNewCopyInPlace(benchmark::State & state)
{
  ConstantVelocityNew model;
  const uint64_t sz = 4U;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  Matrix<float32_t, sz, sz> F;
  for (auto _ : state) {
    F = model.compute_jacobian_in_place(milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}

static void ConstantVelocityNewCreate(benchmark::State & state)
{
  ConstantVelocityNew model;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  for (auto _ : state) {
    const auto F = model.compute_jacobian(milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}

static void ConstantVelocityNewCreateInPlace(benchmark::State & state)
{
  ConstantVelocityNew model;

  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  for (auto _ : state) {
    const auto F = model.compute_jacobian_in_place(milliseconds_100);
    benchmark::DoNotOptimize(F);
  }
}

BENCHMARK(ConstantVelocitySingleAllocation);
BENCHMARK(ConstantVelocityMultipleAllocations);
BENCHMARK(ConstantVelocityNewCopy);
BENCHMARK(ConstantVelocityNewCreate);
BENCHMARK(ConstantVelocityNewCopyInPlace);
BENCHMARK(ConstantVelocityNewCreateInPlace);
BENCHMARK_MAIN();
