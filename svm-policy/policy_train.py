import pickle
import numpy as np
from sklearn import svm
from random_demonstration import *
"""
Original vectors in the paper
state: xa ya za ha la wa cos sin hb wb hc wc
action: px py pz d1x d1y d1z d2x d2y d2z d3x d3y d3z f pre spt1 spt2 spt3

Dummy generated vectors
state: xa ya ha la wa cos sin hb wb hc wc
action: px py pz rx ry rz f pre spt1 spt2 spt3
"""


def filter_demonstrations(state_vectors, action_vectors, vd=0.5, vs=0.5):
    assert state_vectors.shape[0] == action_vectors.shape[
        0], 'Mismatch number of state and action pairs'

    demonstration_vectors = np.concatenate((state_vectors, action_vectors), axis=1)

    demo_clf = svm.OneClassSVM(nu=vd)
    demo_clf.fit(demonstration_vectors)
    demonstrations_score = demo_clf.predict(demonstration_vectors)

    state_clf = svm.OneClassSVM(nu=vs)
    state_clf.fit(state_vectors)
    states_score = state_clf.predict(state_vectors)

    consistent_indices = np.where(np.logical_or(demonstrations_score > 0, states_score < 0))
    c_state_vectors = state_vectors[consistent_indices]
    c_action_vectors = action_vectors[consistent_indices]

    print(f'Filtered {len(state_vectors) - len(c_state_vectors)} inconsistent demonstrations')
    return c_state_vectors, c_action_vectors


def get_regressors(state_vectors, action_vectors, epsilon=0.1):
    assert state_vectors.shape[0] == action_vectors.shape[
        0], 'Mismatch number of state and action pairs'
    assert len(action_vectors.shape) == 2, 'Expected action vector shape to be 2 dimensions'

    regressors = [None] * action_vectors.shape[-1]

    for i in range(len(regressors)):
        regressors[i] = svm.SVR(epsilon=epsilon)
        regressors[i].fit(state_vectors, action_vectors[:, i])

    print('Trained regressors')
    return regressors


def predict_actions(regressors, state_vectors):
    output = np.array([regressors[i].predict(state_vectors) for i in range(len(regressors))])
    output = np.swapaxes(output, 0, 1)
    return output


if __name__ == '__main__':
    from preprocess import *

    state_vectors, action_vectors = load_demo()
    state_vectors, action_vectors = random_gen(state_vectors, action_vectors, 100, 150)
    print(f"loaded: {len(state_vectors)} demonstrations")


    c_state_vectors, c_action_vectors = filter_demonstrations(
        state_vectors,
        action_vectors,
        vd=0.5,
        vs=0.5,
    )


    print("================ Consistent Demonstrations ================")
    regressors_consistent = get_regressors(c_state_vectors, c_action_vectors, epsilon=0.1)

    output = predict_actions(regressors_consistent, c_state_vectors)
    error = output - c_action_vectors
    error = np.linalg.norm(error, 2, axis=1)
    error = np.mean(error)

    print(f'Average error: {error}')

    with open('models/test_demo_weights_consistent_only.model', 'wb') as f:
        pickle.dump(regressors_consistent, f)

    print("================ All Demonstrations ================")

    regressors_all_demonstration = get_regressors(state_vectors, action_vectors, epsilon=0.1)

    output = predict_actions(regressors_all_demonstration, state_vectors)
    error = output - action_vectors
    error = np.linalg.norm(error, 2, axis=1)
    error = np.mean(error)
    print(f'Average error: {error}')

    with open('models/test_demo_weights_all_demos.model', 'wb') as f:
        pickle.dump(regressors_all_demonstration, f)
    