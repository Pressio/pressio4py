
# rom: LSPG: steady problem

@m_class{m-note m-default}

@parblock
Defined in module: `pressio4py.rom.lspg.steady`

Import as: &emsp; &emsp; &emsp; `from pressio4py.rom import lspg`
@endparblock


## API

```py
problem = lspg.steady.Problem(fom_adapter, decoder, \				(1)
						      rom_state, fom_ref_state)

problem = lspg.steady.PrecProblem(fom_adapter, decoder, rom_state,  (2)
								  fom_ref_state, preconditioner)
```

### Parameters and Requirements

- `fom_adapter`:
  - instance of your adapter class specifying the FOM problem. <br/>
  - must satisfy the [steady API](./md_pages_components_rom_fom_apis.html)

- `decoder`:
  - decoder object
  - must satify the requirements listed [here](md_pages_components_rom_decoder.html)

- `rom_state`:
  - currently, must be a rank-1 `numpy.array`

- `fom_ref_state`:
  - your FOM reference state that is used when reconstructing the FOM state
  - must be a rank-1 `numpy.array`

- `preconditioner`:
  - an functor needed to precondition the ROM operators
  - must be a functor with a specific API:
  ```py
  class Prec:
	def __call__(self, fom_state, operand):
	  # given the current FOM state,
	  # apply your preconditioner to the operand.
	  # Ensure that you overwrite the data in the operand.
	  # As an example, a trivial preconditioner that does nothing:
	  # operand[:] *= 1.
  ```


<br/>
___
<br/>


## Example code

```py
import numpy as np
from scipy import linalg
from pressio4py import logger, solvers, rom
from pressio4py.rom import lspg

# ===============================
class MySteadyAdapter:
  def __init__(self, N):
    assert(N==6)
    self.N_ = N

  def createResidual(self):
    return np.zeros(self.N_)

  def createApplyJacobianResult(self, operand):
    return np.zeros_like(operand)

  def residual(self, stateIn, R):
    R[:] = 1.0

  def applyJacobian(self, stateIn, operand, C):
    J = self.jacobian(stateIn)
    C[:]  = J.dot(operand)

  def jacobian(self, stateIn):
    return np.identity(self.N_)

# ===============================
class MyLinSolver:
  def solve(self, A,b,x):
    # solve Ax = b
    # here we should solve the system,
	# but for demonstration let's fix the solution
    x[:] = np.array([1.,2.,3.])

# ===============================
if __name__ == "__main__":
  logger.initialize(logger.logto.terminal)
  logger.setVerbosity([logger.loglevel.debug])

  np.random.seed(334346892)

  N = 6
  appObj = MySteadyAdapter(N)
  yRef = np.ones(N)

  romSize = 3
  phi = np.ones((meshSize, romSize), order='F')
  phi[:,0] = 1
  phi[:,1] = 2
  phi[:,2] = 3
  decoder = rom.Decoder(phi)

  yRom    = np.zeros(romSize)
  problem = lspg.steady.Problem(appObj, decoder, yRom, yRef)

  # linear and non linear solver
  lsO  = MyLinSolver()
  nlsO = solvers.create_gauss_newton(problem, yRom, lsO)
  nlsO.solve(problem, yRom)
  print(yRom)

  fomRecon = problem.fomStateReconstructor()
  yFomFinal = fomRecon(yRom)
  print(yFomFinal)

  logger.finalize()
```
