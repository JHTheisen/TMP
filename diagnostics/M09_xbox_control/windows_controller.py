"""Read Xbox safety controls without SDL's potentially modal Windows event pump.

Never generates motion. B aborts; a deliberate thumbstick displacement cancels auto.
SDL still owns ordinary button actions and fresh manual JOG generation.
"""
import ctypes
import sys


def safety_reader(index, deadband):
    if sys.platform != "win32":
        return None
    try:
        dll = ctypes.WinDLL("xinput1_4")
    except OSError:
        return None

    class Gamepad(ctypes.Structure):
        _fields_ = [("buttons", ctypes.c_ushort), ("lt", ctypes.c_ubyte),
                    ("rt", ctypes.c_ubyte), ("lx", ctypes.c_short),
                    ("ly", ctypes.c_short), ("rx", ctypes.c_short), ("ry", ctypes.c_short)]

    class State(ctypes.Structure):
        _fields_ = [("packet", ctypes.c_ulong), ("pad", Gamepad)]

    get_state = dll.XInputGetState
    get_state.argtypes = [ctypes.c_uint, ctypes.POINTER(State)]
    get_state.restype = ctypes.c_uint
    connected = False

    def poll():
        nonlocal connected
        state = State()
        if get_state(index, ctypes.byref(state)):
            if connected:
                raise RuntimeError("Xbox controller disconnected (XInput)")
            return None
        connected = True
        pad = state.pad
        if pad.buttons & 0x2000:
            return "abort"
        # Read physical thumbsticks, independent of SDL's backend-dependent
        # raw axis numbering. Resting triggers are deliberately excluded.
        if any(abs(value / 32768) > deadband for value in (pad.lx, pad.ly, pad.rx, pad.ry)):
            return "takeover"
        return None

    return poll
