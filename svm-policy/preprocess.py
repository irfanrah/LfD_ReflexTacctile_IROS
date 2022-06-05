import cv2 as cv
import numpy as np
from math import atan2, cos, sin
from constants import *
from utils import *


def extract_image_features(pts, img):
    data_pts = np.array(pts, dtype=np.float64).reshape(len(pts), 2)
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)
    center = (int(mean[0, 0]), int(mean[0, 1]))

    axes = []
    for _ in range(4):
        axes.append({})

    axes[0]['dir'] = normalize_vector(eigenvectors[0])
    axes[1]['dir'] = normalize_vector(eigenvectors[1])

    axes[0]['intersection'] = find_intersection2(center, axes[0]['dir'], img, CONTOUR_COLOR)
    axes[1]['intersection'] = find_intersection2(center, axes[1]['dir'], img, CONTOUR_COLOR)

    axes[0]['len'] = np.linalg.norm(axes[0]['intersection'][0] - axes[0]['intersection'][1])
    axes[1]['len'] = np.linalg.norm(axes[1]['intersection'][0] - axes[1]['intersection'][1])

    major_idx = np.argmax([axes[0]['len'], axes[1]['len']])
    minor_idx = 1 - major_idx

    new_center = (axes[major_idx]['intersection'][1] + axes[major_idx]['intersection'][0]) // 2
    axes[0]['center'] = new_center
    axes[1]['center'] = new_center

    axes[minor_idx]['intersection'] = find_intersection2(
        new_center,
        axes[minor_idx]['dir'],
        img,
        CONTOUR_COLOR,
    )
    axes[minor_idx]['len'] = np.linalg.norm(axes[minor_idx]['intersection'][0] -
                                            axes[minor_idx]['intersection'][1])

    axes[2]['center'] = (new_center + axes[major_idx]['intersection'][0]) // 2
    axes[2]['intersection'] = find_intersection2(
        axes[2]['center'],
        axes[minor_idx]['dir'],
        img,
        CONTOUR_COLOR,
    )
    axes[2]['len'] = np.linalg.norm(axes[2]['intersection'][0] - axes[2]['intersection'][1])
    axes[3]['center'] = (new_center + axes[major_idx]['intersection'][1]) // 2
    axes[3]['intersection'] = find_intersection2(
        axes[3]['center'],
        axes[minor_idx]['dir'],
        img,
        CONTOUR_COLOR,
    )
    axes[3]['len'] = np.linalg.norm(axes[3]['intersection'][0] - axes[3]['intersection'][1])

    if axes[major_idx]['dir'][0] < 0:
        axes[major_idx]['dir'] *= -1
    angle = atan2(axes[major_idx]['dir'][1], axes[major_idx]['dir'][0])

    return axes, major_idx, minor_idx, angle


def draw_axes(img, axes, major_idx, minor_idx):
    cv.circle(img, axes[0]['center'], 2, (255, 0, 255), 2)
    cv.circle(img, axes[2]['center'], 2, (0, 255, 0), 2)
    cv.circle(img, axes[3]['center'], 2, (0, 255, 0), 2)
    cv.circle(img, axes[minor_idx]['intersection'][0], 2, (0, 0, 255), 2)
    cv.circle(img, axes[minor_idx]['intersection'][1], 2, (0, 0, 255), 2)
    cv.circle(img, axes[major_idx]['intersection'][0], 2, (255, 0, 0), 2)
    cv.circle(img, axes[major_idx]['intersection'][1], 2, (255, 0, 0), 2)
    cv.circle(img, axes[2]['intersection'][0], 2, (0, 255, 0), 2)
    cv.circle(img, axes[2]['intersection'][1], 2, (0, 255, 0), 2)
    cv.circle(img, axes[3]['intersection'][0], 2, (0, 255, 0), 2)
    cv.circle(img, axes[3]['intersection'][1], 2, (0, 255, 0), 2)

    cv.line(img, axes[major_idx]['intersection'][0], axes[major_idx]['intersection'][1],
            (255, 0, 0), 1)
    cv.line(img, axes[minor_idx]['intersection'][0], axes[minor_idx]['intersection'][1],
            (0, 0, 255), 1)
    cv.line(img, axes[2]['intersection'][0], axes[2]['intersection'][1], (0, 255, 0), 1)
    cv.line(img, axes[3]['intersection'][0], axes[3]['intersection'][1], (0, 255, 0), 1)


def generate_state_vector(img_path, img_ref_path, threshold=100, invert=True, show_img=False):
    img = cv.imread(img_path)
    img_ref = cv.imread(img_ref_path)
    img_subtract = img - img_ref

    assert img.shape == img_ref.shape, 'Mismatch image and ref image dimensions'

    blank = np.zeros(img_subtract.shape, np.uint8)
    gray = cv.cvtColor(img_subtract, cv.COLOR_BGR2GRAY)

    if invert:
        gray = 255 - gray

    _, thresh = cv.threshold(gray, threshold, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for i, points in enumerate(contours):
        area = cv.contourArea(points)
        if area < AREA_MIN_THRESHOLD or area > AREA_MAX_THRESHOLD:
            continue

        # assume there will be only one object only
        cv.drawContours(blank, contours, i, CONTOUR_COLOR, 2)
        axes, major_idx, minor_idx, angle = extract_image_features(points, blank)

        if show_img:
            draw_axes(blank, axes, major_idx, minor_idx)
            cv.imshow('img', img)
            cv.imshow('img_ref', img_ref)
            cv.imshow('img - img_ref', img - img_ref)
            cv.imshow('gray', gray)
            cv.imshow('thresh', thresh)
            cv.imshow('res', blank)
            cv.waitKey(0)
            cv.destroyAllWindows()
        return np.array([
            axes[major_idx]['center'][0],
            axes[major_idx]['center'][1],
            img[axes[major_idx]['center'][1]][axes[major_idx]['center'][0]][0],
            axes[major_idx]['len'],
            axes[minor_idx]['len'],
            cos(angle),
            sin(angle),
            img[axes[2]['center'][1]][axes[2]['center'][0]][0],
            axes[2]['len'],
            img[axes[3]['center'][1]][axes[3]['center'][0]][0],
            axes[3]['len'],
        ])


if __name__ == '__main__':
    state_vector = generate_state_vector(
        'test_images/7.jpg',
        'test_images/7_ref.jpg',
        50,
        invert=False,
        show_img=True,
    )
    print(state_vector)