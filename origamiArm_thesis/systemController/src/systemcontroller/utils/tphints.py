from typing import Any

import numpy as np

# use the proper type hint when developing, but to run on Jetson have to use "Any";
# it appears that this should be supported on the python version it is running,
# but it does not work

# OpenCVframe = np.ndarray[Any, np.dtype[np.integer[Any] | np.floating[Any]]]

OpenCVframe = Any
