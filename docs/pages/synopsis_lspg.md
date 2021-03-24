

## Synopsis for constructing LSPG problems

This page is in progress with more information to be added.

@m_class{m-note m-success}

CONTINUOUS-TIME API

```py
# default
problem = rom.lspg.unsteady.default.Problem<odekeyword>(appObj,
														decoder,
														romState,
														fomReferenceState)

### Hyper-reduced LSPG
problem = rom.lspg.unsteady.hyperreduced.Problem<odekeyword>(appObj,
															 decoder,
															 romState,
															 fomReferenceState,
															 sampleMeshMappingIndices)

# masked
problem = rom.lspg.unsteady.masked.Problem<odekeyword>(appObj,
													   decoder,
													   romState,
													   fomReferenceState,
													   masker)
```

In the code snippets above, `odekeyword` is needed to identify which time-stepping scheme to use,
and is one of: `Euler, BDF2` (these are thos
currently supported, but more schemes will be added overtime).
Note that LSPG only makes sense for implicit time stepping. <br>
For example:

```py
problem = rom.lspg.default.ProblemEuler(...)
```

<br>

@m_class{m-note m-success}

DISCRETE-TIME API

```py
# default or hyper-reduced
# (for LSPG with discrete-time API, there is no underlying implementation
# difference between default or hyper-reduced since user assembles operators directly)
problem = rom.lspg.unsteady.ProblemDiscreteTime<Two/Three>States(appObj,
																 decoder,
																 romState,
																 fomReferenceState)

# masked
problem = rom.lspg.unsteady.masked.ProblemDiscreteTime<Two/Three>States(appObj,
																		decoder,
																		romState,
																		fomReferenceState,
																		masker)
```

For the discrete-time API, when creating the problem one needs to choose
the number of states needed for the stencil. Currently, we support either two or three.
So when creating the problem, regardless of the problem type, you need
to specify `Two` or `Three`. <br>
For example:

```py
problem = rom.lspg.unsteady.ProblemDiscreteTimeTwoStates(...)
```
