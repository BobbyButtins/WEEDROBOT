# Battery Status
# [no battery yet to test]

status = "/sys/class/power_supply/BAT0/capacity"
with open(status) as f:
    while True:
        power = f.read()
        print(power)

