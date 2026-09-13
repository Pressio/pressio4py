"""Top-level package for pressio4py."""

from importlib.metadata import PackageNotFoundError, version

try:
  __version__ = version("pressio4py")
except PackageNotFoundError:
  # Allows source-tree imports before installation.
  __version__ = "0+unknown"
