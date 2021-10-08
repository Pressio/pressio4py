
# logger



@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.logger`

Import as: &emsp; &emsp; &emsp; `from pressio4py import logger`
@endparblock


## Overview

@m_class{m-note m-warning}

@parblock
By default, for performance reasons, the logger is disabled, so pressio4py will not output anything.
@endparblock

@m_class{m-note m-info}

@parblock
To enable logging, you need to do two things:
1. initialize the logger and (if needed) change the verbosity level
2. finalize the logger
@endparblock

## Usage

The following snippet provides the main idea:

```py
from pressio4py import logger

if __name__ == '__main__':
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.info])

  # your code

  logger.finalize()
```

### Levels

```py
loglevel.trace
loglevel.debug
loglevel.info
loglevel.warn
loglevel.error
loglevel.critical
loglevel.off
```

@m_class{m-block m-warning}

@par Keep in mind:
The log statements issued for a specific level will be printed
*only if* `PRESSIO_LOG_ACTIVE_MIN_LEVEL` is smaller or equal than that level.
If the logger is disabled, the macros are expanded to a no-op.
So it does not cost you anything to place log statements in your code,
because in production mode you can just compile to no-op.
