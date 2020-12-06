
import pytest
import numpy as np
import test_logger_module as m
from pressio4py import logger as logger

def test1():
  logger.initialize(logger.logto.terminal, "null")
  logger.setVerbosity([logger.loglevel.debug])
  m.testTraceLog()
