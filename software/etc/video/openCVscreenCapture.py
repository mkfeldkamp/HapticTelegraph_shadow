# Captures video from a named opencv window
# use getArUcoPixelLocations.py to get a window titled "MyArUco"


import cv2
import numpy as np
import pyautogui

import pygetwindow as gw

# Get a specific window by its title
window_title = "MyArUco"
window = gw.getWindowsWithTitle(window_title)[0]

# Get the window's position and size
x, y, width, height = window.left, window.top, window.width, window.height

# Take a screenshot of the window
screenshot = pyautogui.screenshot(region=(x, y, width, height))



while True:
    # Take a screenshot
    #screenshot = pyautogui.screenshot() # grabs entire screen
    window = gw.getWindowsWithTitle(window_title)[0] # grabs specific window

    # Get the window's position and size
    x, y, width, height = window.left, window.top, window.width, window.height

    # Take a screenshot of the window
    screenshot = pyautogui.screenshot(region=(x, y, width, height))

    # Convert the screenshot to a NumPy array
    frame = np.array(screenshot)

    # Convert the color space
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Display the frame
    cv2.imshow("Screen Recorder", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()