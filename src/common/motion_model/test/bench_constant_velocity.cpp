#include <benchmark/benchmark.h>

static void Bench(benchmark::State & state)
{
  int i{};
  for (auto _ : state) {
    i++;
    benchmark::DoNotOptimize(i);
  }
}

BENCHMARK(Bench);
