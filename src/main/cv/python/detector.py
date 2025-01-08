import cv2
import math
import numpy as np
from target import Target
import skimage.color
import time

class Detector:

    def __init__(self):
        pass

    def bgr_to_rgb(self, image):
        return image[:,:,::-1]
    
    def bgr_to_hsv(self, img): # img with BGR format
        maxc = img.max(-1)
        minc = img.min(-1)

        out = np.zeros(img.shape)
        out[:,:,2] = maxc
        out[:,:,1] = (maxc-minc) / maxc

        divs = (maxc[...,None] - img)/ ((maxc-minc)[...,None])
        cond1 = divs[...,0] - divs[...,1]
        cond2 = 2.0 + divs[...,2] - divs[...,0]
        h = 4.0 + divs[...,1] - divs[...,2]
        h[img[...,2]==maxc] = cond1[img[...,2]==maxc]
        h[img[...,1]==maxc] = cond2[img[...,1]==maxc]
        out[:,:,0] = (h/6.0) % 1.0

        out[minc == maxc,:2] = 0
        return out

    def detectGameElement(self, frame, objectsToDetect: list):
        #frame = frame[120:,:] #crop out top half of frame
        #low_threshold = np.array([0.0442839, 0.1188894, 0.8963528]) #skimage
        low_threshold = np.array([102.24340111, 21.13310021, 177.33784898]) #cv2.cvt 

        #high_threshold = np.array([0.18016434, 1.19385776, 1.0694635]) #skimage
        high_threshold = np.array([121.78736932 305.52047777 293.13095836]) #cv2.cvt

        results = dict(zip(objectsToDetect, [None for i in range(len(objectsToDetect))]))
        colors = {
            "RING": [low_threshold, high_threshold]
        }
        for object in objectsToDetect:
            #p = time.time()
            #Converts the color format of the frame from BGR to HSV for usage with cv2
            #full_test_image = self.bgr_to_rgb(frame)
            #hsv_frame = skimage.color.rgb2hsv(full_test_image)
            hsv_frame = cv2.cvtColor(full_test_image, cv2.COLOR_RGB2HSV)
            #hsv_frame = self.bgr_to_hsv(frame)
            #print("color conversion", str(time.time()-p))

            #Creates a mask which is a frame with only the range of colors inputed
            #p = time.time()
            mask = cv2.inRange(hsv_frame, colors[object][0], colors[object][1])
            #print("simple mask", str(time.time()-p))

            #The following three functinos edits the mask in order to remove potential discrepencies in the frame
            #p = time.time()
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))
            morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            # mask = cv2.medianBlur(mask, 5)

            # mask = np.expand_dims(mask, axis = -1)
            #print("better mask", str(time.time()-p))

            #The below code runs to detect if there is a ring in the given frame.
            if (object == "RING"):
                contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                contours = sorted(contours, key=cv2.contourArea)
                contours = [contour for contour in contours if contour.size > 1000]
                # cnt = None
                # contour = contours[len(contours) -1]
                #last detection is supposed be the biggest because of the sorting function above
                if (len(contours) > 0):
                    tx,ty,tw,th = cv2.boundingRect(contours[len(contours) -1])
                    cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th),
                                         (0, 0, 255), 2)

        # while True:
        #     cv2.imshow("o", frame)
        #     if cv2.waitKey(0):
        #         break

        #cv2.destroyAllWindows()

        if (len(contours) > 0):
            results[object] = Target(contours[len(contours) -1], object)
            return results
        return None
        

    def detectColoredShape(self, array, rgb_col):
        color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
                          'white': [[180, 18, 255], [0, 0, 231]],
                          'red1': [[180, 255, 255], [159, 50, 70]],
                          'red2': [[9, 255, 255], [0, 50, 70]],
                          'green': [[89, 255, 255], [36, 50, 70]],
                          'blue': [[128, 255, 255], [90, 50, 70]],
                          'yellow': [[35, 255, 255], [25, 50, 70]],
                          'purple': [[158, 255, 255], [129, 50, 70]],
                          'orange': [[24, 255, 255], [10, 50, 70]],
                          'gray': [[180, 18, 230], [0, 0, 40]]}
        frame = array
        results = dict(zip(rgb_col, [None for i in range(len(rgb_col))]))

        #print(rgb_col)

        for object in rgb_col:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_frame, np.array(color_dict_HSV[object][1]), np.array(color_dict_HSV[object][0]))
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[:2]
            x = 0
            y = 0
            w = 0
            h = 0
            for contour in contours:
                tx, ty, tw, th = cv2.boundingRect(contour)
                #print(tx, ty, tw, th)
                if (tw * th > w * h and not
                   (tx == 0 and ty == 0 and tw == frame.shape[1] and th == frame.shape[0])):
                    x = tx
                    y = ty
                    w = tw
                    h = th

            results[object] = [x, y, w, h]
            #print("X: %2d, Y: %2d, W: %2d, H: %2d" % (x, y, w, h))
            while True:
                if cv2.waitKey(0):
                    break
            cv2.destroyAllWindows()

            results[object] = Target([x, y, w, h], object)

        return results

    def detectAprilTag(self, array):
        results = []

        return results