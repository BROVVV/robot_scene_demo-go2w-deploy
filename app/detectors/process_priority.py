"""Optional background CPU scheduling for isolated detector processes."""
import os
import shutil


def detector_process_command(command):
    value = os.environ.get("GO2W_ML_NICE", "0")
    try:
        priority = int(value)
    except ValueError as error:
        raise ValueError("GO2W_ML_NICE must be an integer from 0 to 19") from error
    if not 0 <= priority <= 19:
        raise ValueError("GO2W_ML_NICE must be an integer from 0 to 19")
    if not priority:
        return list(command)
    executable = shutil.which("nice")
    if executable is None:
        raise RuntimeError("GO2W_ML_NICE requires the nice executable")
    return [executable, "-n", str(priority), *command]
