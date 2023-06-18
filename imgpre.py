import cv2
import numpy as np
import sys

try:
    MAP_IMAGE_PATH = sys.argv[1]
except IndexError:
    print("Error: please specify an image.")
    exit(0)

try:
    SAVE_PATH = sys.argv[2]
except IndexError:
    SAVE_PATH = ''

def img_resize(image):
    height, width = image.shape[0], image.shape[1]
    # 设置新的图片分辨率框架
    width_new = 450
    height_new = 368
    # 判断图片的长宽比率
    if width / height >= width_new / height_new:
        img_new = cv2.resize(image, (width_new, int(height * width_new / width)))
    else:
        img_new = cv2.resize(image, (int(width * height_new / height), height_new))
    return img_new

image = cv2.imread(MAP_IMAGE_PATH)
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY); #转灰度图
#image = img_resize(image)
ret, image = cv2.threshold(image, 172, 255, cv2.THRESH_BINARY) # 二值化
# for y in range(1, len(image)-1):
#     for x in range(1, len(image[0]) - 1):
#         if image[y][x] == 255 and image[y+1][x] == 0 and image[y-1][x] == 0:
#             image[y][x] = 0
#         if image[y][x] == 255 and image[y][x+1] == 0 and image[y][x-1] == 0:
#             image[y][x] = 0

cv2.imshow('img', image)
if len(SAVE_PATH) != 0:
    cv2.imwrite(SAVE_PATH, image)
key = cv2.waitKey(0)
if key == 27:
    cv2.destroyAllWindows()
    exit()