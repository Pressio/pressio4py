
# rom: Decoder

@m_class{m-note m-default}

@parblock

A key assumption of projection-based ROMs is to approximate
the full-order model (FOM) state, @f$y_{fom}@f$, as:
@f[
y_{fom} = g(y_{rom})
@f]

where @f$y_{rom}@f$ is the reduced state (or generalized coordinates),
and @f$g@f$ is the decoder (or mapping).
@endparblock


<br/>

## Custom Decoder

A custom decoder in pressio4py implements the general mapping above.

This allows one to use an arbitrary function to map the ROM state to the FOM state.

```py
class CustomMapper:
  def __init__(self, fomSize, romSize):
	# attention: the jacobian of the mapping must be column-major oder
	# so that pressio can view it without deep copying it, this enables
	# to keep only one jacobian object around and to call the update
	# method below correctly
	self.jacobian_ = np.zeros((fomSize,romSize), order='F')

  def jacobian(self): return self.jacobian_

  def applyMapping(self, romState, fomState):
	#fomState[:] = whatever is needed
	pass

  def updateJacobian(self, romState):
	# update the self.jacobian_[:,:]
	pass

# create the mapper
myMapper = CustomMapper(10,3)
# to create a custom decoder, one can do
customDecoder = rom.Decoder(myMapper, "MyMapper")
```

### Requirements

- `rom_state_type` : rank-1 `numpy.array`

- `fom_state_type` : rank-1 `numpy.array`

- `jacobian_type`&ensp; : rank-2 `numpy.array`


@m_class{m-note m-info}

@parblock
Note: there is no explicit constraint on what the mapping is, it can be anything.

As long as the decoder (or mapper) class implements the concept, it is admissible.
@endparblock


<br/>
____
<br/>


## Linear Decoder

A linear decoder is a mapping of the form:
@f[
y_{fom} = \phi y_{rom}
@f]

where @f$\phi@f$ is the Jacobian matrix (for the time being, assume it constant).

### Example usage
```py
# create the matrix
# attention: phi must be column-major for these reasons:
#
# 1. pressio4py uses blas (wherever possible) to operate on numpy arrays,
#    so a column-major layout implies seamless compatiblity with blas
#
# 2. when using column-major layout, pressio4py references the
#    matrix phi without doing a deep copy, which saves memory
#    since a single jacobian matrix is alive.
#
phi = np.ones((10,3), order='F')

# to create the linear decoder, you simply do
linearDecoder = rom.Decoder(phi)

# linearDecoder exposes a method to evaluate the mapping
fomState, romState = np.zeros(10), np.ones(3)
linearDecoder.applyMapping(romState, fomState)
print(fomState)
```

### Requirements

- `rom_state_type` : rank-1 `numpy.array`

- `fom_state_type` : rank-1 `numpy.array`

- `jacobian_type`&ensp; : rank-2 `numpy.array`
