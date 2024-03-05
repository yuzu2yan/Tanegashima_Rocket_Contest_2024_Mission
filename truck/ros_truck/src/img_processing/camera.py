import cv2
 
capture = cv2.VideoCapture(0)
 
while(True):
    ret, frame = capture.read()
    # windowsize = (640, 480)
    # frame = cv2.resize(frame, windowsize)
 
    cv2.imshow('camera test',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
capture.release()
cv2.destroyAllWindows()