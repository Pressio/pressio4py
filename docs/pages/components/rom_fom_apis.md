
# Full-order model (FOM) adapter API


@m_class{m-note m-default}

@parblock
*The FOM adapter is the layer standardizing how pressio
queries operators from an application.*

It is one of the most important features and a pivotal design in pressio.
@endparblock


@m_class{m-block m-success}

@par Minimally intrusive, you only need to expose information you already have
Preparing an adapter should only involve *exposing* some operators in your application.
Pressio does NOT require you to provide information that you don't already have.
In fact, pressio needs to access *standard* information/operators that your application
already assembles in some form or another. In some cases, these operators might not
be fully exposed yet, so all we need is for you to make them accessible.
As such, writing an adapter is, in most cases, relatively simple.
Note, also, that writing an adapter for *your* application only involves
operating within *your* application domain, using your data structures
and whatever functionalities your application supports.
Therefore, this adapter *lives within your application space*.
@endparblock


@m_class{m-block m-success}

@par Different adapters for different needs
Depending on what problem you are trying to solve,
we haved designed different adapter concepts/APIs,
e.g. steady, unsteady, exposing only partial information, etc,
that fit different scenarios.
Note that not all adapters can be used for all ROM methods we support.
See below for all the details.
@endparblock

<br/>


# Steady API

@m_class{m-note m-info}

@parblock
Intended for when your FOM application is expressed as
@f[
\boldsymbol{R}(\boldsymbol{y}; \boldsymbol{\mu}) = 0
@f]
where @f$y@f$ is your FOM state, and @f$R@f$ is the residual
\todo finish.
@endparblock

### Synopsis

```py
class SteadyAdapter:

  def createResidual(self):
    return np.zeros(...)

  def createApplyJacobianResult(self, operand):
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    # compute residual

  def applyJacobian(self, stateIn, operand, C):
    # compute apply Jacobian
    # for example:
	#   J = self.jacobian(stateIn)
	#	C[:]  = J.dot(operand)
```

### Notes
@m_class{m-note m-warning}

@parblock
The steady adapter can ONLY be used for doing steady LSPG ROMs.
@endparblock

See the following examples:
\toadd


<br/>
___
<br/>


# Continuous-time API: RHS only

@m_class{m-note m-info}

@parblock
Intended for when your FOM application is expressed in *time-continuous* form as
@f[
\frac{d \boldsymbol{y}}{dt} =
\boldsymbol{f}(\boldsymbol{y},t; \boldsymbol{\mu}),
\quad \boldsymbol{y}(0;\boldsymbol{\mu}) = \boldsymbol{y}(\boldsymbol{\mu}),
@f]
where @f$y@f$ is the full-order model (FOM) state,
@f$f@f$ is what we call the FOM velocity (or RHS), and @f$t@f$ is time,
and, for some reason, you can/want to only expose
the right-hand-side (or velocity) of your FOM application.
\todo finish.
@endparblock

### Synopsis

```py
class ContTimeFomAdapterVelocityOnly:

  # create f(y,t,...)
  def createVelocity():
	# say N is the total number of of unknowns
	return np.zeros(N)

  # compute velocity, f(y,t;...), for a given state, y, and time, t
  def velocity(self, y, t, f):
	f[:] = #compute velocity as needed
```

### Notes
@m_class{m-note m-warning}

@parblock
This adapter can ONLY be used for doing Galerkin ROMs with explicit time stepping.
@endparblock


<br/>
___
<br/>


# Continuous-time API: RHS and Jacobian action

@m_class{m-note m-info}

@parblock
This API is intended for any system expressible in *time-continuous* form as above,
but you expose both the right-hand-side of your FOM application as well as
the action of the velocity's Jacobian on some operand (more on this later).
@endparblock


### Synopsis

```py
class ContTimeFomAdapterWithApplyJacobian

  # create f(y,t,...)
  def createVelocity():
	# say N is the total number of of unknowns
	return np.zeros(N)

  # create result of df/dy*B
  # B is typically a skinny dense matrix (e.g. POD modes)
  def createApplyJacobianResult(self, B):
	return np.zeros((N, B.shape[1]))

  # compute velocity, f(y,t;...), for a given state, y
  def velocity(self, y, t, f):
	f[:] = #compute velocity as needed

  # given current state y(t):
  # compute A=df/dy*B, where B is a skinny dense matrix
  # Note that we just require the *action* of the Jacobian.
  def applyJacobian(self, y, B, t, A):
	A[:,:] = # compute A = df/dy * B as needed
```

### Notes

@m_class{m-note m-warning}

@parblock
- Can be used for doing Galerkin ROMs with explicit and implicit time stepping
- Can be used for LSPG and WLS (note that LSPG and WLS only make sense
for implicit time integration).
@endparblock


<br/>
___
<br/>


# Discrete-time API

@m_class{m-note m-info}

@parblock
This API is intended for any system expressible in a discrete-time form as
@f[
\boldsymbol{R}(\boldsymbol{y_{n+1}}, \boldsymbol{y_{n}}, \boldsymbol{y_{n-1}}, ..., t_{n+1}, dt_{n+1}; ...) = \boldsymbol{0}
@f]
where @f$y@f$ is the full-order model (FOM) state, @f$t@f$ is time, and @f$R@f$ is the residual.
\todo finish.
@endparblock


### Synopsis

```py
class DiscreteTimeFomAdapter

  # create R(...)
  def createDiscreteTimeResidual(self):
	return np.zeros(...)

  def createApplyDiscreteTimeJacobianResult(self, B):
	return np.zeros((..., B.shape[1]))

  def discreteTimeResidual(self, step, time, dt, R, y_np1, y_n, y_nm1 [, y_nm2]):
   R[:] = # compute discrete-time residual

  def applyDiscreteTimeJacobian(self, step, time, dt, B, A, y_np1, y_n, y_nm1 [, y_nm2]):
   A[:,:] = # compute the action A = dR/dy_np1 B
```

### Notes

@m_class{m-note m-warning}

@parblock
- For doing Galerkin *implicit* time stepping.
- For doing LSPG and WLS.
@endparblock


<br/>
___
<br/>


# What can you use where?

As anticipated, not all adapters can be used for all supported ROM methods.
The following table illustrates which APIs are admissible for each method.

|                            | Steady API | Continuous Time API <br/> (RHS only) | Continuous Time API <br/> (RHS, Jacobian action) | Discrete Time API |
|----------------------------|------------|-------------------------------------------|----------------------------------------------------------|-------------------|
| Galerkin Explicit Stepping | NA         | supported                                 | supported                                                | NA                |
| Galerkin Implicit Stepping | NA         | NA                                        | supported                                                | supported         |
| LSPG Unsteady              | NA         | NA                                        | supported                                                | supported         |
| LSPG Steady                | supported  | NA                                        | NA                                                       | NA                |
| WLS Explicit Stepping      | NA         | supported                                 | supported                                                | NA                |
| WLS Implicit Stepping      | NA         | NA                                        | supported                                                | supported         |

Note: for LSPG there is no distinction between explicit and implicit
because LSPG only makes sense for implicit time stepping.
Actually, it can be shown that explicit LSPG is equivalent to explicit Galerkin.

<br/>
___
<br/>


# Frequently Asked Questions

@m_class{m-block m-default}

@par 1. Should I prefer the continuous-time or discrete-time API?
In general, we suggest users to always prefer the continuous-time API because it is more general.
However, there are situations where the discrete-time API is more useful or even necessary.
@endparblock
