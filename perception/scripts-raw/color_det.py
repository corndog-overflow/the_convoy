import cv2
import numpy as np
import time
def process_frame(frame):
    cx = None
    cy = None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([0, 130, 70])
    upper_yellow = np.array([10, 255, 255])#currently finds red
    
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    kernel = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1]]) #smoothing kernel for mask convolution
    mask = cv2.filter2D(mask, -1, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    output = np.zeros_like(frame)
    output[mask > 0] = [255, 255, 255] # of a np matrix of zeroes, turn on corresponding pixels that are in the detection mask
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)#get largers contour
        if (cv2.contourArea(largest_contour) > 300):
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)  # Red dot
                    
                    cv2.line(output, (cx, 0), (cx, frame.shape[0]), (0, 0, 255), 2)
                    print(cx, cy)
            pixels_on = np.where(largest_contour > 255);
            print(pixels_on);
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2) # draw bounding box around the largest contour
    
    return output, cx, cy

def main():
    cap = cv2.VideoCapture(0)  
    desired_fps = 15
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
     
        processed_frame = process_frame(frame)[0]
        
        cv2.imshow("color det", processed_frame)

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
