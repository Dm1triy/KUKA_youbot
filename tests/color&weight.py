import cv2
import numpy as np

path = '/home/kpu/dev/__/KUKA_youbot_latest/debug/floor.jpg'

img = cv2.imread(path)


weights = np.zeros((img.shape[0], img.shape[1]))
purp1 = [128, 0, 128]
purp2 = [75, 0, 130]

orng1 = [255, 69, 0]
orng2 = [255, 165, 0]

# color always in BGR format
cv2.rectangle(img, pt1=(200, 200), pt2=(300, 300), color=orng1, thickness=-1)     # HSV (60°, 60%, 100%)
cv2.rectangle(img, pt1=(400, 100), pt2=(500, 200), color=orng2, thickness=-1)       # HSV (51°, 100%, 100%)

img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


pixel = img[250, 250]   # BGR
print('BGR1', pixel)
pixel = img[150, 450]   # BGR
print('BGR2', pixel)

hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
pixel = hsv[250, 250]   # HSV
print('HSV1', pixel)
pixel = hsv[150, 450]   # HSV
print('HSV2', pixel)

lower_bound = np.array([pixel[0]-10, 45, 45])
upper_bound = pixel + np.array([pixel[0]+10, 255, 255])
mask = cv2.inRange(hsv, lower_bound, upper_bound)

weights = weights + 1.5 * mask/255
print(np.max(weights))

print(img[100, 100])
if (img[100, 100] == [166, 175, 182]).all():
    print("Get it")
print(np.array(img[100, 100]) == [166, 175, 182])

cv2.imshow('Floor', img)
cv2.waitKey(0)
cv2.imshow('Floor', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()