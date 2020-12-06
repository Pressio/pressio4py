
#include "pressio_utils.hpp"
#include "types.hpp"

namespace
{
void testTraceLog()
{
  PRESSIOLOG_TRACE("tracing {:.f}", 2.);
}
}//anonym namespace

PYBIND11_MODULE(test_logger_module, m)
{
  m.def("testTraceLog", &testTraceLog);
}
