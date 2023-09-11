import cv2

cap = cv2.VideoCapture(2)

num = 0

while cap.isOpened():

    succes, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #ret, corners = cv2.findChessboardCorners(gray, (8,5), None)
    # print(corners)

    k = cv2.waitKey(5)

    if k == ord('q'):
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('./images/' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()