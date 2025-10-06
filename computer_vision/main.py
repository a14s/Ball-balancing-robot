import cv2
import numpy as np
import serial
import time

# Serial Setup 
ser = serial.Serial('COM12', 115200, timeout=1) 
# time.sleep(2) 

camera = cv2.VideoCapture(0)

calibrated = False
points = []
px_per_cm = None
pixel_distance = None

last_send = time.time()

# Mouse Callback 
def mouse_callback(event, x, y, flags, param):
    global points, pixel_distance
    if event == cv2.EVENT_LBUTTONDOWN and not calibrated:
        points.append((x, y))
        if len(points) == 2:
            pixel_distance = np.sqrt((points[0][0] - points[1][0])**2 +
                                     (points[0][1] - points[1][1])**2)
            print(f"Pixel distance: {pixel_distance:.2f}px")

cv2.namedWindow("Ball Tracking", cv2.WINDOW_NORMAL)
#cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Ball Tracking", mouse_callback)
cv2.resizeWindow("Ball Tracking", 800, 600)  
#cv2.resizeWindow("Mask", 800, 600)           

def pick_biggest_round_contour(contours, min_area=300, min_circ=0.70):
    """
    Return the contour that is largest while still reasonably round.
    - min_area: reject tiny highlights
    - min_circ: circularity threshold; 1.0 is a perfect circle
    """
    best = None
    best_score = -1.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        peri = cv2.arcLength(c, True)
        if peri == 0:
            continue
        circularity = 4.0 * np.pi * (area / (peri * peri))
        if circularity < min_circ:
            continue
        # score favors larger and rounder blobs
        score = area * circularity
        if score > best_score:
            best_score = score
            best = c
    return best

while True:
    ret, frame = camera.read()
    if not ret:
        print(" Could not read from camera.")
        break

    # --- Calibration visualization ---
    if not calibrated:
        if len(points) > 0:
            cv2.circle(frame, points[0], 4, (0, 255, 0), -1)
            cv2.putText(frame, "1", (points[0][0] + 10, points[0][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if len(points) == 2:
            cv2.circle(frame, points[1], 4, (0, 255, 0), -1)
            cv2.putText(frame, "2", (points[1][0] + 10, points[1][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.line(frame, points[0], points[1], (0, 255, 0), 2)

            if pixel_distance:
                cv2.putText(frame, f"{pixel_distance:.2f} px",
                            ((points[0][0] + points[1][0]) // 2,
                             (points[0][1] + points[1][1]) // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)

        cv2.putText(frame, "Click on 2 points and press 'c' to calibrate",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 1)

    # --- Ball Tracking after calibration ---
    if calibrated:
        # NEW: slight blur to calm sensor noise/highlight specks
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)  # NEW
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Your white range
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # NEW: close small holes and remove pepper noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))  # NEW
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)   # NEW
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # NEW

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # NEW: choose one contour only – the biggest round-ish white blob
        c = pick_biggest_round_contour(contours, min_area=400, min_circ=0.65)  # tweak thresholds if needed
        if c is not None:
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                x, y, r = int(x), int(y), int(radius)

                # reference (0,0) at frame center
                cx = frame.shape[1] // 2
                cy = frame.shape[0] // 2
                cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

                # Convert to cm
                x_cm = (x - cx) / px_per_cm
                y_cm = (y - cy) / px_per_cm

                msg = f"{int(round(x_cm))},{int(round(y_cm))}\n"
                try:
                    ser.write(msg.encode('utf-8'))
                    response = ser.readline().decode('utf-8').strip()
                    print(f"{'Sending:':<10}{msg.strip():<30} {'Received:':<10}{response:<30}")
                except:
                    print("Serial send error.")
                last_send = time.time()

                # Draw chosen ball only
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                cv2.putText(frame, f"Ball: ({x_cm:.2f},{y_cm:.2f}) cm",
                            (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 0), 2)
                cv2.circle(frame, (x, y), 4, (255, 0, 255), -1)
        else:
            # Optional: debug text when no valid ball is found
            cv2.putText(frame, "No round white target found",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        #cv2.imshow("Mask", mask)

    else:
        #cv2.imshow("Mask", np.zeros(frame.shape[:2], dtype=np.uint8))
        temp = 1

    cv2.imshow("Ball Tracking", frame)

    key = cv2.waitKey(10) & 0xFF

    if key == ord('c') and len(points) == 2 and not calibrated:
        print(" Enter the real distance between the two points (cm): ")
        try:
            cm_distance = 16
            if cm_distance > 0:
                px_per_cm = pixel_distance / cm_distance
                calibrated = True
                print(f" Calibration done: {px_per_cm:.2f} px/cm")
            else:
                print(" Distance must be > 0")
                points = []
        except ValueError:
            print(" Please enter a valid number")
            points = []

    if key == ord('r'):
        points = []
        calibrated = False
        px_per_cm = None
        pixel_distance = None
        print(" Reset done.")

    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
