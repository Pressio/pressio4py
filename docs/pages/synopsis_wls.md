

## Synopsis for constructing WLS problems

This page is in progress with more information to be added.

@m_class{m-note m-success}

CONTINUOUS-TIME API

```py
# Default WLS
wlsPolicy = rom.exp.wls.default.SequentialPolicyBDF1(romSize, numStepsInWindow, decoder, appObj, yRef)
wlsSystem = rom.exp.wls.default.ProblemBDF1(decoder, wlsPolicy, yFomInitCond, yRef, wlsState)

wlsPolicy = rom.exp.wls.default.SequentialPolicyBDF2(romSize, numStepsInWindow, decoder, appObj, yRef)
wlsSystem = rom.exp.wls.default.ProblemBDF2(decoder, wlsPolicy, yFomInitCond, yRef, wlsState)

# hyper-reduced
Implementation in progress.

# masked
Implementation in progress.
```

<br>


@m_class{m-note m-success}

DISCRETE-TIME API


```py
# default
wlsPolicy = rom.exp.wls.default.SequentialPolicyDiscreteTimeBDF1(romSize, numStepsInWindow, decoder, appObj, yRef)
wlsSystem = rom.exp.wls.default.ProblemDiscreteTimeBDF1(decoder, wlsPolicy, yFomInitCond, yRef, wlsState)

wlsPolicy = rom.exp.wls.default.SequentialPolicyDiscreteTimeBDF2(romSize, numStepsInWindow, decoder, appObj, yRef)
wlsSystem = rom.exp.wls.default.ProblemDiscreteTimeBDF2(decoder, wlsPolicy, yFomInitCond, yRef, wlsState)

# hyper-reduced
Implementation in progress.

# masked
Implementation in progress.
```
