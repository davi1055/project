global log_level

def setLogLevel(level):
    assert type(level) == type(""), f"\033[31mlevel value must be a string"
    global log_level
    log_level = level


def log(msg, level_name="DEBUG"):
    global log_level
    assert type(level_name) == type(""), "\033[31mlevel_name value must be a string"
    level = {"UNKNOW": 0,"DEBUG": 32, "INFO": 97, "WARN": 33, "ERROR": 31, "FATAL": 41}

    # When level_name is less than log_level, the log does not print the information
    # Tip, traversal in reverse order
    for key in reversed(level):
        if key == log_level:
            return
        if key == level_name:
            break

    if level_name not in level:
        print(
            f"\033[33m{msg}(Please use level_name:{[key in level]})\033[0m"
        )
        return
    print(f"\033[{level[level_name]}m{msg}\033[0m")
