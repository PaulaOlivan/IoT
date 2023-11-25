import board
import busio
from digitalio import DigitalInOut
from adafruit_pn532.i2c import PN532_I2C
reset_pin = DigitalInOut(board.D6)

i2c = busio.I2C(board.SCL, board.SDA)
pn532 = PN532_I2C(i2c, debug=False, reset=reset_pin, req=req_pin)

ic, ver, rev, support = pn532.firmware_version
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

pn532.SAM_configuration()
while True:
    uid = pn532.read_passive_target(timeout=0.5)
    print('.', end="", flush=True)
    if uid is None:
        continue
    print('Found card with UID:', [hex(i) for i in uid])