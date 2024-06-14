import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM numbering
GPIO.setmode(GPIO.BOARD)

input_1 = 15
input_2 = 33
input_3 = 40
input_4 = 18

test_pin = input_3
# Set up GPIO pin 18 as an input
GPIO.setup(test_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

POLL_INTERVAL = 0.01 # Polling interval in seconds (10ms)
DEBOUNCE_COUNT = 2 # Number of consecutive polls needed to confirm switch press 

def is_switch_pressed():
    return GPIO.input(test_pin) == GPIO.HIGH

try:
    consecutive_presses = 0
    while True:
        # Read the state of the GPIO pin
        if is_switch_pressed():
            consecutive_presses += 1
            if consecutive_presses >= DEBOUNCE_COUNT:
                print("Limit switch pressed")
                consecutive_presses = 0 # Reset the counter after the event has occured
        else:
            consecutive_presses = 0

        time.sleep(POLL_INTERVAL) # wait before next poll

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()

