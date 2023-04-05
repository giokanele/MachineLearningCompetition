
import threading
from pynput import keyboard
import numpy as np

import time 


# A variable that will be modified by the user input
var = "" 
# boolean to send message between threads
capture = threading.Event()

# The function that will be run continuously in the background
def main():
    global capture
    init = True
    while True:
        # Do something continuously
        if capture.is_set():
            if init == True: #initalize data collection
                myData = []
                value = 0
                init = False
            if init == False:
                value += 1 
                myData.append(value)  #append data every frame 
                print(value)
                time.sleep(1)  #for this example only
        else:
            if init == False:   #if init just turned off, 
                array = np.array(myData)
                print("Example data", array) 
                # save numpy array instead
                init = True
        continue

def background_function():
    # Start the background thread
    # Accept user input from the command prompt
    while True:
        with keyboard.Events() as events:
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('r'):
                if capture.is_set():
                    capture.clear()
                    print("\nStopped")
                    time.sleep(0.2) #wait a bit until next input
                else:
                    print("\nRecording")
                    capture.set()
                    time.sleep(0.2)
                    
            

if __name__ == '__main__':
    background_thread = threading.Thread(target=background_function)
    background_thread.daemon = True
    background_thread.start()
    capture.clear()
    data = 0
    main()


