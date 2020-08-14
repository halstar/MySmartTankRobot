import psutil


def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value


def is_process_running(name):

    for process in psutil.process_iter():
        try:
            if name == process.name():
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    return False;
