import board
import digitalio

led = digitalio.DigitalInOut(board.D25)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = False