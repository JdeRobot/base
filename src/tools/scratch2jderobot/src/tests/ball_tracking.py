# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
import imutils
import cv2

# define the lower and upper boundaries of the basic colors
GREEN_RANGE = ((29, 86, 6), (64, 255, 255))
RED_RANGE = ((139, 0, 0), (255, 160, 122))
BLUE_RANGE = ((0, 128, 128), (65, 105, 225))


def centroid(frame, color):
    # resize the frame
    frame = imutils.resize(frame, width=600)

    # convert to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color specified
    # then perform a series of dilations and erosions
    # to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, color[0], color[1])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and
    # initialize the current center
    cnts = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle border
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            # and the centroid
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    return center


if __name__ == "__main__":
    color = GREEN_RANGE

    # if a video path was not supplied, grab the reference
    # to the webcam
    camera = cv2.VideoCapture(0)

    while True:
        # grab the current frame
        (grabbed, frame) = camera.read()

        center = centroid(frame, color)
        if center != None:
            print center
        else:
            print "NO OBJECT"

    # cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()
