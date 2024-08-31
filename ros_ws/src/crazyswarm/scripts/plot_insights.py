import matplotlib.pyplot as plt
max_iter = 6
R_prefix = "force_estimation_error_RRLS_xy_"
L_prefix = "force_estimation_error_LRLS_xy_"
DT = 0.01


import pickle
import numpy as np
import matplotlib.pyplot as plt


def load_ndarray_from_pickle(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)

def compute_row_norms(ndarray):
    return np.linalg.norm(ndarray, axis=1)

if __name__ == "__main__":
    # List all pickle files in the directory
    R_pickle_files = []
    L_pickle_files = []
    for i in range(1, max_iter):
        R_pickle_files.append(R_prefix + "{}.pkl".format(i))
        L_pickle_files.append(L_prefix + "{}.pkl".format(i))

    # Initialize a plot
    fig = plt.figure(figsize=(20, 8), frameon=True, dpi=200)
    ax = fig.add_subplot(111)

    # Iterate through sorted files, load, compute norms, and plot
    R_ndarray = load_ndarray_from_pickle(R_prefix + ".pkl")
    R_row_norms = compute_row_norms(R_ndarray)
    L_ndarray = load_ndarray_from_pickle(R_prefix + ".pkl")
    L_row_norms = compute_row_norms(L_ndarray)
    x_values = [i * DT for i in range(len(L_ndarray))]

    for iter, R_filename, L_filename in zip(range(max_iter-1), R_pickle_files, L_pickle_files):
        R_ndarray = load_ndarray_from_pickle(R_filename)
        R_row_norms = np.concatenate([R_row_norms, compute_row_norms(R_ndarray)])
        L_ndarray = load_ndarray_from_pickle(L_filename)
        L_row_norms = np.concatenate([L_row_norms, compute_row_norms(L_ndarray)])
        x_values += [(i + (iter+1)*len(L_ndarray)) * DT for i in range(len(L_ndarray))]
        

    ax.plot(x_values, L_row_norms, label='left')
    ax.plot(x_values, R_row_norms, label='right')
    ax.set_ylim(0, 0.4)
    plt.xlabel('Time (s)')
    plt.ylabel('Force estimation Error (N)')
    plt.legend()
    plt.grid(True)
    plt.show()