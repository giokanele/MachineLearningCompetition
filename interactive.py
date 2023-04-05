import threading

# A variable that will be modified by the user input
user_input = ""
capture = threading.Event()

# The function that will be run continuously in the background
def main():
    global user_input
    global capture
    while True:
        # Do something continuously
        if capture.is_set():
            print("Multithreading is awesome!")
            capture.clear()
        
        continue

def background_function():
    # Start the background thread
    # Accept user input from the command prompt
    while True:
        value = input("Enter something: ")
        if(value == "q"):
            capture.set()
        else: 
            user_input = value

        # Do something with the user input
        print(f"User input received: {user_input}")


if __name__ == '__main__':
    background_thread = threading.Thread(target=background_function)
    background_thread.daemon = True
    background_thread.start()
    main()


