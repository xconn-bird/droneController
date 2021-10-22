import time
import board
import adafruit_hcsr04
sonar = adafruit_hcsr04.HCSR04(trigger.DS, echo_pin =board.D6)

while True:
    try:
        print((sonar.distance,))
        except RuntimeError:
            print("Retrying!")
            time.sleep(0.1)
