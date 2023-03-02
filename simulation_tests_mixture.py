import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
from glob import glob

import calibration.utils.io
from calibration.calibration import calibrate
from calibration.utils.tools import get_relative_error, get_absolute_error

path = './simulation_tests/mixture'

algorithms = ['Park', 'Taylor', 'Zhuang', 'DNL', 'DNLO', 'Ali']

methods = ['A', 'B', 'C']
ns = [5, 10]


# Initial guess for Zuang
def initial_rho(R, t):
    rX = Rotation.from_matrix(R).as_rotvec().squeeze()
    w = np.linalg.norm(rX)
    kX = rX / w if w else np.array([0, 0, 1])
    return np.hstack((np.tan(w/2) * kX, t))


rho = initial_rho(np.eye(3), [0, 0, 0])

results = []

for f in glob(path + '/run_*'):
    GT = calibration.utils.io.get_trajectories(
        [f + '/T_S2_S1.txt'])[0].as_array().squeeze()
    trajectories = calibration.utils.io.get_trajectories(
        [f + '/T_S1_W1.txt', f + '/T_S2_W2.txt'], estimate=False)

    for method in methods:
        nn = [1] if method == 'A' else [1, *ns] if method == 'B' else ns
        for n in nn:
            trajectories_rel = [T.relative_pose(method, n) for T in trajectories]

            for algorithm in algorithms:
                params = {
                    'algorithm' : algorithm,
                    'method'    : method,
                    'n'         : n,
                    'scale'     : False
                }
                if 'Zhuang' in algorithm or 'detmax' in algorithm:
                    params['initial_guess'] = rho
                elif 'ransac' in algorithm:
                    params |= {
                        'min_data'    : 3,
                        'max_iter'    : 100,
                        'threshold'   : (2, 0.05),
                        'min_inliers' : .3 * len(trajectories[0])
                    }

                T, s, cost = calibrate(trajectories[0], trajectories[1], **params)

                try:
                    err_tr, err_R = get_relative_error(trajectories_rel[0],
                                                       trajectories_rel[1],
                                                       T)
                    abs_tr, abs_R = get_absolute_error(T, GT)
                except:
                    err_tr, err_R = None, None
                    abs_tr, abs_R = None, None

                result = {
                    'algorithm'  : algorithm,
                    'method'     : method + str(n),
                    'err_tr'     : err_tr,
                    'err_R'      : err_R,
                    'abs_tr'     : abs_tr,
                    'abs_R'      : abs_R,
                    'T'          : T
                }
                results.append(result)

df = pd.DataFrame(results)
df.to_csv('calib/simulation_tests_mixture.csv')
