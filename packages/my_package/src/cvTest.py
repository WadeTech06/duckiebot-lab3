import cv2
import numpy as np 

""" # Load an color image
img = cv2.imread('D:/WadeTech/Documents/University of Sheffield/Robotics and Autonomous Systems/duckiebot-lab3/packages/my_package/src/LW/images/20200313_130651.jpg',1)
imgS = cv2.resize(img, (600, 600)) 

# set red range and mask of red colored objects
lower_red = np.array([17, 15, 100])
upper_red = np.array([50, 56, 200]) 

red_mask = cv2.inRange(imgS,lower_red,upper_red)

result = cv2.bitwise_and(imgS,imgS, mask= red_mask)
gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
count = cv2.countNonZero(gray)
print(count)
cv2.imshow('images',np.hstack([imgS,result])) 

cv2.waitKey(0)
cv2.destroyAllWindows() """


# Load an color image
img = cv2.imread('D:/WadeTech/Documents/University of Sheffield/Robotics and Autonomous Systems/duckiebot-lab3/packages/my_package/src/LW/images/20200313_130651.jpg',1)
imgS = cv2.resize(img, (600, 600)) 

# set yellow range and mask of yellow colored objects
lower_yellow = np.array([0, 153, 153])
upper_yellow = np.array([102, 255, 255]) 

yellow_mask = cv2.inRange(imgS,lower_yellow,upper_yellow)

result = cv2.bitwise_and(imgS,imgS, mask= yellow_mask)
gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

# get the image dimensions
r = gray.shape[0]
c = gray.shape[1]
rightlimit = int(c*.6)
count=0
# loop over the image, pixel by pixel
for y in range(0, r):
    for x in range(rightlimit, c):
        # threshold the pixel
        if gray[y, x] > 0:
            count +=1

print(count)
cv2.imshow('images',np.hstack([imgS,result])) 

cv2.waitKey(0)
cv2.destroyAllWindows()