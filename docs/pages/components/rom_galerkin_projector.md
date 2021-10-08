
# rom: Galerkin: projector

The projector is needed for Galerkin to perform the projection of the
FOM operators onto the reduced space.


@m_class{m-note m-info}

@parblock
It is explicitly required from the user when doing [maked](md_pages_components_rom_galerkin_masked.html)
or [hyper-reduced](md_pages_components_rom_galerkin_hypred.html) Galerkin.
@endparblock

For a [default](md_pages_components_rom_galerkin_default.html) problem,
you don't need to pass it because the projector is constructed behind
the scenes automatically using the decoder's jacobian.

\todo: explain more, talk about pressio-tools.

## API

When provided by the user, the projector operator must be
provided in the form of a functor as follows:

```py
class Projector:
  def __init__(self, ...):
    # as needed

  def __call__(self, operand, time, result):
    # project operand and store in result
```

Note that the operand is either a FOM velocity instance, or the decoder's Jacobian.
In all cases, however, it is a `numpy.array`.
You can put define the actual projection operation however you like.

One thing to keep in mind is that, typically, the `operand` is either a masked
operand (i.e., the result of masking a full FOM operand) if you are using
a masked problem, or it is a hyper-reduced object if you are using a
hyper-reduced Galerkin problem.
