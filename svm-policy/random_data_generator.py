import numpy as np
import math

np.random.seed(2022)


def gen_random_state_vector():
    xa = np.random.rand() * 5
    ya = np.random.rand() * 5
    ha = np.random.randn() * 1
    la = np.random.randn() * 5
    wa = np.random.randn() * 3
    hb = ha - np.random.randn() * .05
    wb = wa - np.random.randn() * .05
    hc = wa - np.random.randn() * .05
    wc = wa - np.random.randn() * .05
    angle = np.random.rand() * math.pi * 2
    return [xa, ya, ha, la, wa, math.cos(angle), math.sin(angle), hb, wb, hc, wc]


def gen_random_action_vector():
    px = np.random.rand() * 5
    py = np.random.rand() * 5
    pz = np.random.rand() * 5
    rx = np.random.rand() * 0.02
    ry = np.random.rand() * 0.02
    rz = np.random.rand() * 0.02
    f = np.random.rand() * .5
    pre = np.random.rand() * .5
    spt1 = np.random.rand() * 20
    spt2 = np.random.rand() * 20
    spt3 = np.random.rand() * 20
    return [px, py, pz, rx, ry, rz, f, pre, spt1, spt2, spt3]


if __name__ == '__main__':
    import pickle

    state_vectors = []
    action_vectors = []

    for i in range(600):
        state_vect = gen_random_state_vector()
        state_vectors.append(state_vect)

        action_vect = gen_random_action_vector()
        action_vectors.append(action_vect)

        with open('dummy_data/state_vectors.demo', 'wb') as f:
            pickle.dump(state_vectors, f)

        with open('dummy_data/action_vectors.demo', 'wb') as f:
            pickle.dump(action_vectors, f)