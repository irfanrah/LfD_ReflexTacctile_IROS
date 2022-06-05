import os
import json
import cv2 as cv
import numpy as np
from constants import *


def rotate(img, angle, rot_center=None):
    if rot_center is None:
        rot_center = (img.shape[1] // 2, img.shape[0] // 2)
    rot_mat = cv.getRotationMatrix2D(rot_center, angle, 1.0)
    return cv.warpAffine(img, rot_mat, img.shape[:2])


def is_pixel_color_same(pixel1, pixel2):
    return pixel1[0] == pixel2[0] and pixel1[1] == pixel2[1] and pixel1[2] == pixel2[2]


def is_out_of_bound(pos, max_x, max_y):
    return pos[0] < 0 or pos[1] < 0 or pos[0] >= max_x or pos[1] >= max_y


def normalize_vector(vect):
    return vect / np.linalg.norm(vect)


def find_intersection2(
    center,
    direction,
    img,
    contour_color,
    max_iteration=INTERSECTION_MAX_ITERATION,
):

    first_point = center + direction * max_iteration
    last_point = center - direction * max_iteration

    points = np.linspace(first_point, last_point, int(np.linalg.norm(last_point - first_point)))
    points = np.array([p for p in points if not is_out_of_bound(p, img.shape[1], img.shape[0])],
                      dtype=np.int32)

    candidates = []
    for point in points:
        if is_pixel_color_same(img[point[1]][point[0]], contour_color):
            candidates.append(point)

    return np.array(candidates[0]), np.array(candidates[-1])


def find_intersection(
    center,
    direction,
    img,
    contour_color,
    neighbor_size=NEIGHBOR_SIZE,
    max_iteration=INTERSECTION_MAX_ITERATION,
):
    candidates = []

    test_center = [center[0], center[1]]
    iter = 0

    while iter < max_iteration and not is_out_of_bound(test_center, img.shape[1], img.shape[0]):
        for i in range(-neighbor_size, neighbor_size + 1):
            pos_y = test_center[1] + i

            if pos_y < 0 or pos_y >= img.shape[0]:
                continue

            for j in range(-neighbor_size, neighbor_size + 1):
                pos_x = test_center[0] + j

                if pos_x < 0 or pos_x >= img.shape[1]:
                    continue

                if is_pixel_color_same(img[pos_y][pos_x], contour_color):
                    candidates.append((pos_x, pos_y))

        test_center[0] = int(test_center[0] + direction[0] * neighbor_size * 3)
        test_center[1] = int(test_center[1] + direction[1] * neighbor_size * 3)
        iter += 1

    index = 0
    if len(candidates) > 1:
        vect_dot_product = []
        for candidate in candidates:
            vect = [candidate[0] - center[0], candidate[1] - center[1]]
            vect = normalize_vector(vect)
            vect_dot_product.append(np.abs(np.dot(vect, direction)))

        index = np.argmax(vect_dot_product)

    return candidates[index]


def load_demo(demo_dir='demonstrations'):
    demos = os.listdir(demo_dir)

    state_vectors = []
    action_vectors = []

    for demo in demos:
        if not os.path.isdir(os.path.join(demo_dir, demo)):
            continue

        path = os.path.join(demo_dir, demo, 'state-action.txt')
        f = open(path)
        state_action = json.load(f)
        state_vect = np.array(state_action['state'])
        action_vect = np.array([
            *state_action['p'],
            state_action['roll'],
            state_action['pitch'],
            state_action['yaw'],
            *state_action['f'],
            state_action['pre'],
            np.average(state_action['spt1']),
            np.average(state_action['spt2']),
            np.average(state_action['spt3']),
        ])
        state_vectors.append(state_vect)
        action_vectors.append(action_vect)

    state_vectors = np.array(state_vectors)
    action_vectors = np.array(action_vectors)

    return state_vectors, action_vectors