from machine import Pin, UART
from time import sleep
# import ulab


led_board = Pin(2, Pin.OUT)  # onboard LED
print("Starting in 5 seconds... Close Thonny and start Webots.")
for i in range(5):
    led_board.value(not led_board())
    sleep(1)
led_board.value(0)

# Now switch to UART1 for communication with Webots
uart = UART(1, 115200, tx=1, rx=3)

# Initial status of the line sensor: updated by Webots via serial
line_left = False
line_center = False
line_right = False

# Variables to implement the line-following state machine
current_state = 'forward'
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
state_updated = True

while True:
    
    ##################   See   ###################
    
    # Check if anything was received via serial to update sensor status
    if uart.any():
        try:
            msg_line = uart.readline()  # safer: read one line
            msg_str = msg_line.decode('utf-8').strip()

            if len(msg_str) == 3 and all(c in '01' for c in msg_str):
                line_left   = msg_str[0] == '1'
                line_center = msg_str[1] == '1'
                line_right  = msg_str[2] == '1'
            else:
                print("Invalid message received:", msg_str)
        except Exception as e:
            print("UART read error:", e)



    ##################   Think   ###################

    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center:
            current_state = 'turn_left'
            state_updated = True

    elif current_state in ['turn_right', 'turn_left']:
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'stop':
        led_board.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            led_board.value(0)
            state_updated = True

            
    
    ##################   Act   ###################

    # Send the new state when updated
    if state_updated:
        print("Sending to Webots:", current_state)
        uart.write(current_state + '\n')
        state_updated = False

    counter += 1    # increment counter
    sleep(0.02)     # wait 0.02 seconds
   

