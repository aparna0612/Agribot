import cv2
import numpy as np

'''pass --> is used to do nothing'''
'''a is just a necessary argument'''

def empty(a):
    pass


cap = cv2.VideoCapture(1)
cap.set(3,200)
cap.set(4,200)
cap.set(10,150)

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",250,250)
#Hue Min is the name of the tracker
'''Trackbars is the name of the window on 
which we want to place the track bars'''
'''0 is the initial value of the pointer and
179 is the max value of the pointer'''
'''empty is basically a function that is called
whenever there is a change in the values of the 
HUE'''


cv2.createTrackbar("Hue Min","TrackBars",25,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",60,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",63,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",255,255,empty)
cv2.createTrackbar("Val Min","TrackBars",151,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)

while True:

    success,img = cap.read()

    #to get the HSV --> Hue Saturation Value of the image

    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
    v_min = cv2.getTrackbarPos("Val Min","TrackBars")
    v_max = cv2.getTrackbarPos("Val Max","TrackBars")

    print(h_min,h_max,s_min,s_max,v_min,v_max)
    '''the lower and the upper are used to set a
    range'''
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    #this is to create a mask
    '''this will filter out and give us the 
    image of that colour'''
    mask = cv2.inRange(imgHser,upper)

    '''Now we are creating a new image using the 
    mask'''

    '''This statement states that the new image 
    will be like our image but with a mask applied'''
    imgResult = cv2.bitwise_and(img,img,mask=mask)

    cv2.imshow("Image",img)

    cv2.imshow("HSV Image",imgHsv)

    cv2.imshow("Mask",mask)

    cv2.imshow("Result",imgResult)

    if cv2.waitKey(1) & 0xFF == ord('q'):