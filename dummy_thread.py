import threading
import time

def Thread1():
    print("GPS Thread1")

def Thread2():
    print("Calculation Thread2")

def Thread3():
    print("Main Loop Thread3")

i = 0
val = True

# Define a function for the main loop
def main_loop():
    global val  # Use the global keyword to modify the global val variable
    global i
    while i < 10:
        Thread3()
        i += 1
        time.sleep(1)  # Adjust the sleep duration as per your desired frequency
    val = False

def GPS_loop():
    while val:
        Thread1()
        time.sleep(3)

def calcLoop():
    while val:
        Thread2()
        time.sleep(5)

main_thread = threading.Thread(target=main_loop)
gps_thread = threading.Thread(target=GPS_loop)
cal_thread = threading.Thread(target=calcLoop)

main_thread.start()
gps_thread.start()
cal_thread.start()