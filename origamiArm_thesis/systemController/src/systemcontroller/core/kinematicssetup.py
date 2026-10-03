import ctypes


def setup(
    *,
    buffer_size: int,
    motorl_xy_mm: tuple[int | float, int | float],
    motorr_xy_mm: tuple[int | float, int | float],
    link_lengths_mm: tuple[float, float, float, float],  # link lengths left to right
) -> ctypes.CDLL:
    """Sets up the external kinematics shared library ready for use."""
    print("Setting up shared dynamics library")
    klib: ctypes.CDLL = _setup_kinematicslib_interface(buff_size=buffer_size)
    print(
        f"Link lengths are set to {link_lengths_mm[0]}mm, {link_lengths_mm[1]}mm, "
        f"{link_lengths_mm[2]}mm, {link_lengths_mm[3]}mm"
    )
    klib.set_up(
        ctypes.c_double(link_lengths_mm[0]),
        ctypes.c_double(link_lengths_mm[1]),
        ctypes.c_double(link_lengths_mm[2]),
        ctypes.c_double(link_lengths_mm[3]),
        ctypes.c_double(motorl_xy_mm[0]),  # left motor x coordinate
        ctypes.c_double(motorl_xy_mm[1]),  # left motor y coordinate
        ctypes.c_double(motorr_xy_mm[0]),  # right motor x coordinate
        ctypes.c_double(motorr_xy_mm[1]),  # right motor y coordinate
    )
    return klib


def _setup_kinematicslib_interface(*, buff_size: int) -> ctypes.CDLL:
    """Loads the shared kinematics library and sets up its wrapper."""
    kinlib = ctypes.CDLL("./kinematicsExtLib.so")  # load the shared library file
    kinlib.set_up.argtypes = [
        ctypes.c_double,  # link1 (LHS) mm
        ctypes.c_double,  # link2 mm
        ctypes.c_double,  # link3 mm
        ctypes.c_double,  # link4 (RHS) mm
        ctypes.c_double,  # motor 1 x position
        ctypes.c_double,  # motor 1 y position
        ctypes.c_double,  # motor 2 x position
        ctypes.c_double,  # motor 2 y position
    ]
    kinlib.get_steplists.argtypes = [
        ctypes.POINTER(ctypes.c_int * buff_size),  # left_list
        ctypes.POINTER(ctypes.c_int * buff_size),  # right_list
        ctypes.c_int,  # max_listsize
        ctypes.POINTER(ctypes.c_int),  # actual_listsize
        ctypes.c_int,  # curr_x_coord
        ctypes.c_int,  # curr_y_coord
        ctypes.c_int,  # desired_x_coord
        ctypes.c_int,  # desired_y_coord
    ]
    kinlib.get_steplists.restype = ctypes.c_bool  # true if found route, else false
    return kinlib
