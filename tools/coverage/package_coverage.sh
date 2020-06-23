#!/usr/bin/env bash

usage_exit() {
	echo "Usage: ${0} PACKAGE_NAME" 1>&2
	exit 1
}

COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"

if [ $# -eq 0 ]
	then
		usage_exit
fi

declare -a dirs=("build" "install" "log" "lcov")
for i in "${dirs[@]}"
do
	if [ -d "$i" ]; then
		read -p "'$i' directory exists. Delete it? " -n 1 -r
		echo
		if [[ $REPLY =~ [Yy]$ ]]
		then
			rm -rf $i
		fi
	fi
done

if [ ! -d lcov ]; then
	mkdir lcov
fi

# Build with correct flags
colcon build \
  --merge-install \
  --packages-up-to $1 \
  --cmake-args \
		-DCMAKE_BUILD_TYPE=Coverage \
	--ament-cmake-args \
	  -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
	  -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" || { echo "Build failed." ; exit 1; }

# Get a zero-coverage baseline
lcov \
	--base-directory "$(pwd)" \
	--capture \
	--initial \
	--directory "$(pwd)/build/$1" \
	-o "$(pwd)/lcov/lcov.base" \
 	--no-external --rc lcov_branch_coverage=1 || { echo "Zero baseline coverage failed."; exit 1; }

# Run unit and integration tests
colcon test \
	--merge-install \
  --packages-select $1 \
	--abort-on-error \
	--ctest-args -E __rmw_micro_dds_c\|pclint\|copyright\|cppcheck\|cpplint\|flake8\|lint_cmake\|pep257\|uncrustify || { echo "Unit/integration testing failed."; exit 1; }

# Get coverage
lcov \
  --base-directory "$(pwd)" \
  --capture \
  --directory "$(pwd)/build/$1" \
  --output-file "$(pwd)/lcov/lcov.run" \
  --no-external \
  --rc lcov_branch_coverage=1 || { echo "Coverage generation failed."; exit 1; }

# Combine zero-coverage with coverage information.
lcov \
  -a "$(pwd)/lcov/lcov.base" \
  -a "$(pwd)/lcov/lcov.run" \
  -o "$(pwd)/lcov/lcov.total" \
  --rc lcov_branch_coverage=1 || { echo "Coverage combination failed."; exit 1; }

# Filter test, build, and install files and generate html
lcov -r "$(pwd)/lcov/lcov.total" \
    "*/build/*" \
    "*/install/*" \
    "CMakeCCompilerId.c" \
    "CMakeCXXCompilerId.cpp" \
    -o "$(pwd)/lcov/lcov.total.filtered" \
    --rc lcov_branch_coverage=1 || { echo "Filtering failed."; exit 1; }

genhtml -o "$(pwd)/lcov/" "$(pwd)/lcov/lcov.total.filtered" -p "$(pwd)" --legend \
--demangle-cpp --rc genhtml_branch_coverage=1 || { echo "HTML generation failed."; exit 1; }

echo "Check lcov/index.html to see the coverage report."
