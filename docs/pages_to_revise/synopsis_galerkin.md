
## Synopsis for constructing a Galerkin problem

This page is in progress with more information to be added.

@m_class{m-note m-success}

CONTINUOUS-TIME API

```py
# default
problem = rom.galerkin.default.Problem<odekeyword>(fomObj,
												   decoder,
												   romState,
												   fomReferenceState)
# hyper-reduced
problem = rom.galerkin.hyperreduced.Problem<odekeyword>(fomObj,
														decoder,
														romState,
														fomReferenceState,
														projector)
# masked
problem = rom.galerkin.masked.Problem<odekeyword>(fomObj,
												  decoder,
												  romState,
												  fomReferenceState,
												  masker,
												  projector)
```

In the code snippets above, `odekeyword` is needed to identify which time-stepping scheme to use,
and is one of: `ForwardEuler, RK4, AB2, BackwardEuler, BDF2` (these are thos
currently supported, but more schemes will be added overtime). <br>
For example:

```py
problem = rom.galerkin.default.ProblemRK4(...)
```

<br>


@m_class{m-note m-success}

DISCRETE-TIME API

```py
# default
problem = rom.galerkin.default.ProblemDiscreteTime<Two/Three>States(appObj,
																	decoder,
																	romState,
																	fomReferenceState)
# hyper-reduced
problem = rom.galerkin.hyperreduced.ProblemDiscreteTime<Two/Three>States(appObj,
																		 decoder,
																		 romState,
																		 fomReferenceState,
																		 projector)
# masked
problem = rom.galerkin.masked.ProblemDiscreteTime<Two/Three>States(appObj,
																   decoder,
																   romState,
																   fomReferenceState,
																   masker,
																   projector)
```

For the discrete-time API, when creating the problem one needs to choose
the number of states needed for the stencil. Currently, we support either two or three.
So when creating the problem, regardless of the problem type, you need to either use `Two` or `Three`. <br>
For example, to create a default Galerkin for the discrete-time API with two stencil states, one would do:

```py
problem = rom.galerkin.masked.ProblemDiscreteTimeTwoStates(...)
```
