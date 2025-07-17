import cv2
import numpy as np

# 이진 이미지 (0 또는 255)
binary = cv2.imread('binary_image.png', cv2.IMREAD_GRAYSCALE)

# 라벨링 수행
num_labels, labels_im = cv2.connectedComponents(binary)

print('블롭 개수:', num_labels - 1)  # 0은 배경

# 각 블롭의 좌표, 면적 정보 얻기
for label in range(1, num_labels):  # 0은 배경이므로 1부터
    mask = np.uint8(labels_im == label)
    x, y, w, h = cv2.boundingRect(mask)
    area = cv2.countNonZero(mask)
    print(f'블롭 {label}: 위치=({x}, {y}), 크기=({w}, {h}), 면적={area}')
