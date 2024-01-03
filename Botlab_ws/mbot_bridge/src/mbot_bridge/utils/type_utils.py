import importlib
import mbot_lcm_msgs


class BadMessageError(Exception):
    pass


def str_to_lcm_type(dtype):
    """Accesses the message type by string. Raises AttributeError if the class type is invalid."""
    # Check whether the dtype is a full path to the package.
    if "." in dtype:
        pkg, msg_type = dtype.split(".")  # The data type should be pkg.msg_type_t.
        pkg = importlib.import_module(pkg)
        return getattr(pkg, msg_type)

    # By default, try to load the type from mbot_lcm_msgs.
    return getattr(mbot_lcm_msgs, dtype)


def decode(data, dtype):
    """Decode raw data from LCM channel to type based on type string."""
    try:
        lcm_obj = str_to_lcm_type(dtype)
    except (ValueError, AttributeError, ModuleNotFoundError) as e:
        raise BadMessageError(f"Could not parse dtype {dtype}: {e}")
    return lcm_obj.decode(data)


def lcm_type_to_dict(data):
    """LCM types, once decoded"""
    data_d = {att: getattr(data, att) for att in data.__slots__}
    return data_d


def dict_to_lcm_type(data, dtype):
    try:
        lcm_msg = str_to_lcm_type(dtype)()
    except (ValueError, AttributeError, ModuleNotFoundError) as e:
        raise BadMessageError(f"Could not parse dtype {dtype}: {e}")

    for k, v in data.items():
        # If one of the values is a list, we must check for types that need to be converted recursively.
        if isinstance(v, list):
            val_dtype = lcm_msg.__typenames__[lcm_msg.__slots__.index(k)]
            if val_dtype.startswith("mbot_lcm_msgs"):
                val_dtype = val_dtype.replace("mbot_lcm_msgs.", "")
                # Recursively convert to the correct data type.
                v = [dict_to_lcm_type(val, val_dtype) for val in v]

        setattr(lcm_msg, k, v)

    return lcm_msg
