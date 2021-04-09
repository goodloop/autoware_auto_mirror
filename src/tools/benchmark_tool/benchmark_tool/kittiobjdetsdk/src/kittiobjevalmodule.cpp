// Copyright (c) 2020-2021, Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdio.h>
#include <strings.h>

#include <iostream>
#include <string>

namespace kittisdk
{
extern "C" {
extern int eval(
  std::string ground_truth_path,
  std::string detection_path,
  std::string output_path,
  bool eval_2d_res,
  bool eval_ground_red,
  bool eval_3d_res,
  bool print_stdout = false,
  bool create_plot = false
);
}
}  // namespace kittisdk

static PyObject *
kittiobjeval_eval(PyObject * self, PyObject * args)
{
  const char * ground_truth_path;
  const char * detection_path;
  const char * output_path;
  bool eval_2d_res;
  bool eval_ground_res;
  bool eval_3d_res;
  bool print_stdout = false;
  bool create_plot = false;
  int sts;

  if (!PyArg_ParseTuple(
      args, "sssbbb|bb", &ground_truth_path,
      &detection_path, &output_path, &eval_2d_res, &eval_ground_res,
      &eval_3d_res, &print_stdout, &create_plot) )
  {
    return NULL;
  }
  sts = kittisdk::eval(
    std::string(ground_truth_path),
    std::string(detection_path),
    std::string(output_path),
    eval_2d_res,
    eval_ground_res,
    eval_3d_res,
    print_stdout,
    create_plot);
  return PyBool_FromLong(sts);
}


static PyMethodDef kittiobjevalMethods[] = {
  {"eval", kittiobjeval_eval, METH_VARARGS,
    "Evaluate using kitti object detection evaluation."},
  {NULL, NULL, 0, NULL}          /* Sentinel */
};


static struct PyModuleDef kittiobjeval_module = {
  PyModuleDef_HEAD_INIT,
  "kittiobjeval",       // name of module
  NULL,                 // module documentation, may be NULL
  -1,                   // size of per-interpreter state of the module,
                        // or -1 if the module keeps state in global variables.
  kittiobjevalMethods
};


PyMODINIT_FUNC
PyInit_kittiobjeval(void)
{
  return PyModule_Create(&kittiobjeval_module);
}
