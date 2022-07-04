import cv2
import numpy as np
import pyfirmata
import time

straight = 7
right = 8
left = 4
back = 10
hatch = 11
addressChange = 13

port = "COM3"

board = pyfirmata.Arduino(port)

it = pyfirmata.util.Iterator(board)
it.start()


##############################
board.digital[straight].write(0)
board.digital[right].write(0)
board.digital[left].write(0)
board.digital[back].write(0)
board.digital[hatch].write(0)
board.digital[addressChange].write(0)
##############################



##############################
'''Functions required'''
def straightMotion():
    board.digital[straight].write(1)



def stopstraightMotion():
    board.digital[straight].write(0)
    time.sleep(1)


def turnRight(val):
    if val == 0:
        board.digital[right].write(1)
        time.sleep(0.5)
    if val == 1:
        board.digital[right].write(1)
        time.sleep(0.62)
    board.digital[right].write(0)
    time.sleep(0.1)


def turnLeft(val):
    if val == 0:
        board.digital[left].write(1)
        time.sleep(0.63)
    if val == 1:
        board.digital[left].write(1)
        time.sleep(0.516)
    board.digital[left].write(0)
    time.sleep(0.1)


def hatchFunction():
    board.digital[hatch].write(1)
    time.sleep(1)
    board.digital[hatch].write(0)
    time.sleep(0.1)



def addressChangeFunction():
    board.digital[addressChange].write(1)
    time.sleep(0.1)
    board.digital[addressChange].write(0)
    time.sleep(0.1)


##############################




#################################
'''Stage variable required'''
stage1 = True
stage2 = False
stage2right = False
stage2left = False
stage3 = False
stage4 = False
stage4hatchOpen = False
stage4hatchClose = False
stage4turnBack = False
stage5 = False
stage6 = False
stage6right = False
stage6left = False
stage7 = False
stage8 = False


################
'''To end the program'''
stage9 = False
################


'''Threshold marking'''
# thresholdValue = 400                                    #Update

'''Initial Boundary'''
initialBoundary = 561



'''Initial boundary state'''
initialBoundaryState = False


'''Current color'''
currentColor = 0
#################################



################################
'''Bot variable'''
bot0 = 1
bot1 = 2
bot2 = 3
bot3 = 4
################################



###############################
'''Box definitions'''
bot0lowerBoundary = 182
# bot1lowerBoundary = 152
# bot2lowerBoundary =
# bot3lowerBoundary =
#
bot0upperBoundary = 120
# bot1upperBoundary = 65
# bot2upperBoundary =
# bot3upperBoundary =
#
bot0rightBoxBoundary = 309
# bot1rightBoxBoundary = 315
# bot2rightBoxBoundary =
# bot3rightBoxBoundary =
#
bot0leftBoxBoundary = 374
# bot1leftBoxBoundary = 394
# bot2leftBoxBoundary =
# bot3leftBoxBoundary =
###############################




###############################
'''Destination definitions'''
bot0rightBoundary = 23
# bot1rightBoundary = 65
# bot2rightBoundary =
# bot3rightBoundary =
#
bot0leftBoundary = 78
# bot1leftBoundary = 120
# bot2leftBoundary =
# bot3leftBoundary =
###############################




cap = cv2.VideoCapture(1)
cap.set(3, 600)
cap.set(4, 600)
cap.set(10, 150)

##############################
'''Red Color, Green color'''
myColors = [[0, 132, 88, 6, 255, 255],
            [69,55,60,89,255,249]]
##############################


def findColor(img, colorValue):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    x, y = 0, 0
    for color in myColors:
        lower = np.array(color[0:3])
        upper = np.array(color[3:6])
        mask = cv2.inRange(imgHSV, lower, upper)

        if color == myColors[colorValue]:
            x, y = getContours(mask)

    return x, y


def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        if area > 500:
            peri = cv2.arcLength(cnt, True)

            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

            x, y, w, h = cv2.boundingRect(approx)

            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (255, 0, 0), 1)

    return x + w // 2, y + h // 2  # for mid of the box


while True:
    success, img = cap.read()
    cv2.imshow("Video", img)
    imgContour = img.copy()
    x, y = findColor(img,currentColor)
    print(x, y)


    # if stage9:          # for the termination of the program
    #     break
    #
    #
    #
    if currentColor == 0:
        if stage1 and x != 0 and y != 0:
            straightMotion()

        if x != 0 and y != 0 and x <= (bot0lowerBoundary+bot0upperBoundary)//2 and stage1 == True :
            stage1 = False
            stage2 = True
            stopstraightMotion()

        if bot0upperBoundary <= x <= bot0lowerBoundary and stage2 == True and x != 0 and y != 0:
            if not stage2right:
                stage2right = True
                stage3 = True
                stage2 = False
                turnRight(0)

        if stage3 and x != 0 and y != 0:
            straightMotion()

        if x != 0 and y != 0 and y <= (bot0leftBoundary+bot0rightBoundary)//2 and stage3 == True : #depends on the position of camera
            stage3 = False
            stage4 = True
            stopstraightMotion()

        if bot0rightBoundary <= y <= bot0leftBoundary and stage4 == True and x != 0 and y != 0:
            if not stage4hatchOpen:
                stage4hatchOpen = True
                stage4hatchClose = True
                hatchFunction()


        if bot0rightBoundary <= y <= bot0leftBoundary and stage4 == True and stage4hatchClose == True and x !=0 and y != 0:
            if not stage4turnBack:
                turnLeft(0)
                time.sleep(0.1)
                turnLeft(0)
                time.sleep(0.1)
                stage4turnBack = True
                stage5 = True
                stage4 = False

        if stage5 == True and x != 0 and y != 0:
            straightMotion()

        if bot0rightBoxBoundary <= y <= bot0leftBoxBoundary and stage5 == True and x != 0 and y != 0:
            stage5 = False
            stage6 = True
            stopstraightMotion()

        if bot0rightBoxBoundary <= y <= bot0leftBoxBoundary and stage6 == True and x != 0 and y != 0:
            if not stage6left:
                turnLeft(0)
                stage6left = True
                stage6 = False
                stage7 = True

        if stage7 == True and x != 0 and y != 0:
            straightMotion()

        if x >= initialBoundary and stage7 == True and x != 0 and y != 0:
            stopstraightMotion()
            initialBoundaryState = True
            stage7 = False
            stage8 = True

        if stage8 == True and initialBoundaryState == True:
            stage1 = True
            stage2 = False
            stage2right = False
            stage2left = False
            stage3 = False
            stage4 = False
            stage4hatchOpen = False
            stage4hatchClose = False
            stage4turnBack = False
            stage5 = False
            stage6 = False
            stage6right = False
            stage6left = False
            stage7 = False
            stage8 = False
            initialBoundaryState = False
            currentColor = 1
            addressChangeFunction()
            continue




    # if currentColor == 1:
    #     if stage1 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and x <= (bot1lowerBoundary+bot1upperBoundary)//2 and stage1 == True :
    #         stage1 = False
    #         stage2 = True
    #         stopstraightMotion()
    #
    #
    #     if bot1upperBoundary <= x <= bot1lowerBoundary and stage2 == True and x != 0 and y != 0:
    #         if not stage2right:
    #             stage2right = True
    #             stage3 = True
    #             stage2 = False
    #             turnRight(1)
    #
    #     if stage3 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and y <= (bot1leftBoundary+bot1rightBoundary)//2 and stage3 == True : #depends on the position of camera
    #         stage3 = False
    #         stage4 = True
    #         stopstraightMotion()
    #
    #     if bot1rightBoundary <= y <= bot1leftBoundary and stage4 == True and x != 0 and y != 0:
    #         if not stage4hatchOpen:
    #             stage4hatchOpen = True
    #             stage4hatchClose = True
    #             hatchFunction()
    #
    #
    #     if bot1rightBoundary <= y <= bot1leftBoundary and stage4 == True and stage4hatchClose == True and x !=0 and y != 0:
    #         if not stage4turnBack:
    #             turnLeft(1)
    #             time.sleep(0.1)
    #             turnLeft(1)
    #             time.sleep(0.1)
    #             stage4turnBack = True
    #             stage5 = True
    #             stage4 = False
    #
    #     if stage5 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if bot1rightBoxBoundary <= y <= bot1leftBoxBoundary and stage5 == True and x != 0 and y != 0:
    #         stage5 = False
    #         stage6 = True
    #         stopstraightMotion()
    #
    #     if bot1rightBoxBoundary <= y <= bot1leftBoxBoundary and stage6 == True and x != 0 and y != 0:
    #         if not stage6left:
    #             turnLeft(1)
    #             stage6left = True
    #             stage6 = False
    #             stage7 = True
    #
    #     if stage7 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x >= initialBoundary and stage7 == True and x != 0 and y != 0:
    #         stopstraightMotion()
    #         initialBoundaryState = True
    #         stage7 = False
    #         stage8 = True
    #
    #     if stage8 == True and initialBoundaryState == True:
    #         stage1 = True
    #         stage2 = False
    #         stage2right = False
    #         stage2left = False
    #         stage3 = False
    #         stage4 = False
    #         stage4hatchOpen = False
    #         stage4hatchClose = False
    #         stage4turnBack = False
    #         stage5 = False
    #         stage6 = False
    #         stage6right = False
    #         stage6left = False
    #         stage7 = False
    #         stage8 = False
    #         thresholdState = False
    #         initialBoundaryState = False
    #         currentColor = 2
    #         addressChangeFunction()
    #         continue
    #
    #
    #
    #
    #
    #
    # if currentColor == 2:
    #     if stage1 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and x <= (bot2lowerBoundary+bot2upperBoundary)//2 and stage1 == True :
    #         stage1 = False
    #         stage2 = True
    #         stopstraightMotion()
    #
    #
    #     if bot2upperBoundary <= x <= bot2lowerBoundary and stage2 == True and x != 0 and y != 0:
    #         if not stage2right:
    #             stage2right = True
    #             stage3 = True
    #             stage2 = False
    #             turnLeft()
    #
    #     if stage3 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and y <= (bot2leftBoundary+bot2rightBoundary)//2 and stage3 == True : #depends on the position of camera
    #         stage3 = False
    #         stage4 = True
    #         stopstraightMotion()
    #
    #     if bot2rightBoundary <= y <= bot2leftBoundary and stage4 == True and x != 0 and y != 0:
    #         if not stage4hatchOpen:
    #             stage4hatchOpen = True
    #             stage4hatchClose = True
    #             hatchFunction()
    #
    #
    #     if bot2rightBoundary <= y <= bot2leftBoundary and stage4hatchClose == True and x !=0 and y != 0:
    #         if not stage4turnBack:
    #             turnRight()
    #             time.sleep(0.1)
    #             turnRight()
    #             time.sleep(0.1)
    #             stage4turnBack = True
    #             stage5 = True
    #             stage4 = False
    #
    #     if stage5 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if bot2rightBoxBoundary <= y <= bot2leftBoxBoundary and stage5 == True and x != 0 and y != 0:
    #         stage5 = False
    #         stage6 = True
    #         stopstraightMotion()
    #
    #     if bot2rightBoxBoundary <= y <= bot2leftBoxBoundary and stage6 == True and x != 0 and y != 0:
    #         if not stage6left:
    #             turnRight()
    #             stage6left = True
    #             stage6 = False
    #             stage7 = True
    #
    #     if stage7 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x >= initialBoundary and stage7 == True and x != 0 and y != 0:
    #         stopstraightMotion()
    #         initialBoundaryState = True
    #         stage7 = False
    #         stage8 = True
    #
    #     if stage8 == True and initialBoundaryState == True:
    #         stage1 = True
    #         stage2 = False
    #         stage2right = False
    #         stage2left = False
    #         stage3 = False
    #         stage4 = False
    #         stage4hatchOpen = False
    #         stage4hatchClose = False
    #         stage4turnBack = False
    #         stage5 = False
    #         stage6 = False
    #         stage6right = False
    #         stage6left = False
    #         stage7 = False
    #         stage8 = False
    #         thresholdState = False
    #         initialBoundaryState = False
    #         currentColor = 3
    #         addressChangeFunction()
    #         continue
    #
    #
    #
    #
    # if currentColor == 3:
    #     if stage1 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and x <= (bot3lowerBoundary+bot3upperBoundary)//2 and stage1 == True :
    #         stage1 = False
    #         stage2 = True
    #         stopstraightMotion()
    #
    #
    #     if bot3upperBoundary <= x <= bot3lowerBoundary and stage2 == True and x != 0 and y != 0:
    #         if not stage2right:
    #             stage2right = True
    #             stage3 = True
    #             stage2 = False
    #             turnLeft()
    #
    #     if stage3 and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x != 0 and y != 0 and y <= (bot3leftBoundary+bot3rightBoundary)//2 and stage3 == True : #depends on the position of camera
    #         stage3 = False
    #         stage4 = True
    #         stopstraightMotion()
    #
    #     if bot3rightBoundary <= y <= bot3leftBoundary and stage4 == True and x != 0 and y != 0:
    #         if not stage4hatchOpen:
    #             stage4hatchOpen = True
    #             stage4hatchClose = True
    #             hatchFunction()
    #
    #
    #     if bot3rightBoundary <= y <= bot3leftBoundary and stage4hatchClose == True and x !=0 and y != 0:
    #         if not stage4turnBack:
    #             turnRight()
    #             time.sleep(0.1)
    #             turnRight()
    #             time.sleep(0.1)
    #             stage4turnBack = True
    #             stage5 = True
    #             stage4 = False
    #
    #     if stage5 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if bot3rightBoxBoundary <= y <= bot3leftBoxBoundary and stage5 == True and x != 0 and y != 0:
    #         stage5 = False
    #         stage6 = True
    #         stopstraightMotion()
    #
    #     if bot3rightBoxBoundary <= y <= bot3leftBoxBoundary and stage6 == True and x != 0 and y != 0:
    #         if not stage6left:
    #             turnRight()
    #             stage6left = True
    #             stage6 = False
    #             stage7 = True
    #
    #     if stage7 == True and x != 0 and y != 0:
    #         straightMotion()
    #
    #     if x >= initialBoundary and stage7 == True and x != 0 and y != 0:
    #         stopstraightMotion()
    #         initialBoundaryState = True
    #         stage7 = False
    #         stage8 = True
    #
    #     if stage8 == True and initialBoundaryState == True:
    #         stage1 = True
    #         stage2 = False
    #         stage2right = False
    #         stage2left = False
    #         stage3 = False
    #         stage4 = False
    #         stage4hatchOpen = False
    #         stage4hatchClose = False
    #         stage4turnBack = False
    #         stage5 = False
    #         stage6 = False
    #         stage6right = False
    #         stage6left = False
    #         stage7 = False
    #         stage8 = False
    #         thresholdState = False
    #         initialBoundaryState = False
    #         stage9 = True






    cv2.imshow("Contour", imgContour)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break