import numpy as np


class TextColors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    RESET = '\033[0m'
    

def combine_behaviors(behavior_list=[], force_dict={}, use_prioritized_acc=False, **kwargs):
    ''' Calls behaviours, and computes the net weighted acceleration. '''
    max_acc = kwargs.get('max_acc', 0.1)
    all_behaviors = kwargs.get('all_behaviors', ['_formation', '_separation', '_steer_to_avoid'])

    acc = np.array([0., 0.])
    for behavior in behavior_list:
        # print("Focrce: ", behavior, force_dict)
        if list(behavior.keys())[0] not in all_behaviors:
            continue

        acc_request = force_dict[list(behavior.keys())[0]]
        if list(behavior.keys())[0] == '_steer_to_avoid' and not (acc_request[0] != 0.0 and acc_request[1] != 0.0):
            continue

        if use_prioritized_acc:
            acc_mag = np.linalg.norm(acc_request)
            if np.linalg.norm(acc) + acc_mag > max_acc:
                # Clip the acceleration
                return np.clip(acc + acc_request, -max_acc, max_acc)
        acc += acc_request
    return acc