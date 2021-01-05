
#include "pressio_utils.hpp"
#include "types.hpp"

namespace
{
void testTraceLog()
{
  pybind11::print("hello");
  PRESSIOLOG_TRACE("tracing");
}
}//anonym namespace

PYBIND11_MODULE(test_logger_module, m)
{
  m.def("testTraceLog", &testTraceLog);
}
