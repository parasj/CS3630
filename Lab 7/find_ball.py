#!/usr/bin/env python3

import cv2
import sys
import copy

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit('install Pillow to run this code')


def find_ball(opencv_image, debug=False):        
    
    best_circle = None
    
    ## START SOLUTION 
    
    #normalize image, map brightest pixel to 255 and smalles pixel val to 0. 
    [minVal, maxVal, minLoc, maxLoc] = cv2.minMaxLoc(opencv_image)     
    mult = 255/(maxVal-minVal)
    normalized_image = opencv_image - minVal
    normalized_image = np.uint8(mult * normalized_image)

    #blur image
    normalized_image = cv2.GaussianBlur(normalized_image,(7,7),0)
    
    #find candidate circles
    circles = cv2.HoughCircles(normalized_image,cv2.HOUGH_GRADIENT,1,90,
                                param1=30,param2=30,minRadius=10,maxRadius=0)
    
    if circles is not None:

        #round values
        circles = np.uint16(np.around(circles))     
        
        circles = circles[0,:]
    
        best_circle = select_best_circle(normalized_image, circles)            
     
        if debug:
            
            print("Found ", len(circles), " candidate circles")
            display_circles(normalized_image, circles, best_circle)
                                       
            print("Best ball: ", best_circle)
 
        return best_circle
        
    else:
        if debug:
            print("No candidate circles found")
    
    ## END SOLUTION    
    
    return best_circle
    
def select_best_circle(opencv_image, circles):
    best_circle = None
    best_circle_intensity = 50
    best_circle_radius = 0
    for i in circles:
                
        #throw out balls that are high (hack, should account for head angle)
        if i[1] < 40:
            break

        intensity = get_circle_intensity(opencv_image, i)

        if intensity <= best_circle_intensity and i[2] > best_circle_radius:
            best_circle = i   
            best_circle_radius = i[2]
            best_circle_intensity = intensity    

    return best_circle


def get_circle_intensity(opencv_image, circle):
    
    #get image size
    height,width = opencv_image.shape            
    
    mask_img = np.zeros((height,width), np.uint8)
    cv2.circle(mask_img,(circle[0],circle[1]),circle[2],1,thickness=-1)
    masked_data = cv2.bitwise_and(opencv_image, opencv_image, 
                                  mask=mask_img)
    hist_mask = cv2.calcHist([opencv_image],[0],masked_data,
                             [256],[0,256])

    avg_intensity = 0
    total = 0
    
    for h in range(hist_mask.size):
        avg_intensity += h* hist_mask[h]
        total += hist_mask[h]
    
    avg_intensity = avg_intensity/total   
    return avg_intensity

def display_circles(opencv_image, circles, best=None):
    
    #make a copy of the image to draw on
    circle_image = copy.deepcopy(opencv_image)
    circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
    
    for c in circles:
        # draw the outer circle
        cv2.circle(circle_image,(c[0],c[1]),c[2],(255,255,0),2)
        # draw the center of the circle
        cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),3) 
        # write coords
        cv2.putText(circle_image,str(c),(c[0],c[1]),cv2.FONT_HERSHEY_SIMPLEX,
                    .5,(255,255,255),2,cv2.LINE_AA)            
    
    #highlight the best circle in a different color
    if best is not None:
        # draw the outer circle
        print("hit")
        cv2.circle(circle_image,(best[0],best[1]),best[2],(255,0,255), 2)
        # draw the center of the circle
        cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
        # write coords
        cv2.putText(circle_image,str(best),(best[0],best[1]),cv2.FONT_HERSHEY_SIMPLEX,
                    .5,(255,255,255),2,cv2.LINE_AA)            
        
    
    #display the image
    pil_image = Image.fromarray(circle_image)
    pil_image.show()             
      
if __name__ == "__main__":
    pass