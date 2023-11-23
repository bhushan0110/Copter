import threading
import time

def perform_calculations():
    # Your calculation logic here
    result = 2 + 2
    return result

def call_function_based_on_calculation(result):
    if result > 4:
        print("Calling function A")
        # Call your function A here
    else:
        print("Calling function B")
        # Call your function B here

# Define a function for the main loop
def main_loop():
    while True:
        result = perform_calculations()
        call_function_based_on_calculation(result)
        time.sleep(10)  # Adjust the sleep duration as per your desired frequency

main_thread = threading.Thread(target=main_loop)
main_thread.start()