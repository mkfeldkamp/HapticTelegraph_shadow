import cv2.aruco as aruco 
import cv2
import numpy as np


# Acquire video source
#cap = cv2.VideoCapture('myVideo.mp4')
cap = cv2.VideoCapture(0) # use this for live sources like webcams; 0, 1, 2 ...
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640*2)  #knock down the *2 to *1 if your webcam can't support 780p
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480*2)
# cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print ("ERROR: Video source Failed to open...")
    exit()
print ("Actual video Size: %d x %d  @ %d  fps " % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS)))


aruco_dict = aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # Choose an ArUco dictionary ID
parameters = cv2.aruco.DetectorParameters()
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # find the corners (in pixel space) and ID's of all aruco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
  
    # if any markers are found, process them...
    if np.all(ids != None):
        aruco.drawDetectedMarkers(frame, corners) #Draw A square around and markers, red dot is 0-th corner
        
        # look at each marker (it's id and 0-th corner)
        for i, id in enumerate(ids):
                
            print (id) # should be same as ids[i]
            print (corners[i][0][0])  # should be same corner (red dot?)
            ###### DRAW ID #####
            frame = cv2.putText(frame, f"Id: {id}", tuple(corners[i][0][0].astype(int)) , font, 1, (0,255,0),2,cv2.LINE_AA)            
           
        # i = i + 1
        # Display the resulting frame no matter what
    cv2.imshow('MyArUco',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break # out of while() loop

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

