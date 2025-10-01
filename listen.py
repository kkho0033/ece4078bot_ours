import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, Ki, KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250  # PWM units per second (adjust this value to tune ramp speed)
MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'

pid_left_base, pid_right_base = 0, 0

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    
    # Check for actual state change. Without this, false positive happens due to electrical noise
    # After testing, debouncing not needed
    if (prev_left_state is not None and current_state != prev_left_state):       
        left_count += 1
        prev_left_state = current_state
    
    elif prev_left_state is None:
        # First reading
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state, prev_right_time
    current_state = GPIO.input(RIGHT_ENCODER)
    
    if (prev_right_state is not None and current_state != prev_right_state): 
        right_count += 1
        prev_right_state = current_state
        
    elif prev_right_state is None:
        prev_right_state = current_state
    
def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement
    
    # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement  == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)
    
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
    
    
def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

# def pid_control():
#     # Only applies for forward/backward, not turning
#     global left_pwm, right_pwm, left_count, right_count
#     global use_PID, KP, KI, KD, prev_movement, current_movement
#     global pid_left_base, pid_right_base

#     integral = 0
#     last_error = 0
#     last_time = monotonic()

#     # Ramping variables & params
#     ramp_left_pwm = 0
#     ramp_right_pwm = 0
#     previous_left_target = 0
#     previous_right_target = 0

#     while running:
#         current_time = monotonic()
#         dt = current_time - last_time
#         last_time = current_time

#         # Detect current movement mode
#         prev_movement = current_movement
#         if (left_pwm > 0 and right_pwm > 0):
#             current_movement = 'forward'
#         elif (left_pwm < 0 and right_pwm < 0):
#             current_movement = 'backward'
#         elif (left_pwm == 0 and right_pwm == 0):
#             current_movement = 'stop'
#         else:
#             current_movement = 'turn'

#         # If we *enter* straight motion (or change straight direction), snapshot baselines for PID
#         if current_movement in ('forward', 'backward') and prev_movement != current_movement:
#             pid_left_base = left_count
#             pid_right_base = right_count
#             integral = 0
#             last_error = 0

#         if not use_PID:
#             target_left_pwm = left_pwm
#             target_right_pwm = right_pwm
#         else:
#             if current_movement in ('forward', 'backward'):
#                 # Use relative counts since segment start (no global reset needed)
#                 error = (left_count - pid_left_base) - (right_count - pid_right_base)

#                 proportional = KP * error
#                 integral += KI * error * dt
#                 integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
#                 derivative = KD * (error - last_error) / dt if dt > 0 else 0
#                 correction = proportional + integral + derivative
#                 correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
#                 last_error = error

#                 if current_movement == 'backward':
#                     correction = -correction

#                 target_left_pwm  = left_pwm  - correction
#                 target_right_pwm = right_pwm + correction
#             else:
#                 # Turning or stopped: keep encoders intact (NO reset), but clear PID accumulators
#                 integral = 0
#                 last_error = 0
#                 target_left_pwm  = left_pwm
#                 target_right_pwm = right_pwm

#         # Ramping (unchanged)
#         if use_ramping and use_PID:
#             max_change_per_cycle = RAMP_RATE * dt

#             left_diff = target_left_pwm - ramp_left_pwm
#             right_diff = target_right_pwm - ramp_right_pwm

#             left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
#             right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

#             left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
#             right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

#             if left_direction_change:
#                 ramp_left_pwm = target_left_pwm
#             if right_direction_change:
#                 ramp_right_pwm = target_right_pwm

#             if not left_direction_change and not right_direction_change:
#                 if left_needs_ramp or right_needs_ramp:
#                     if abs(left_diff) <= max_change_per_cycle:
#                         ramp_left_pwm = target_left_pwm
#                     else:
#                         ramp_left_pwm += max_change_per_cycle if left_diff > 0 else -max_change_per_cycle

#                     if abs(right_diff) <= max_change_per_cycle:
#                         ramp_right_pwm = target_right_pwm
#                     else:
#                         ramp_right_pwm += max_change_per_cycle if right_diff > 0 else -max_change_per_cycle
#                 else:
#                     ramp_left_pwm = target_left_pwm
#                     ramp_right_pwm = target_right_pwm

#             previous_left_target = target_left_pwm
#             previous_right_target = target_right_pwm
#         else:
#             ramp_left_pwm = target_left_pwm
#             ramp_right_pwm = target_right_pwm

#         final_left_pwm  = apply_min_threshold(ramp_left_pwm,  MIN_PWM_THRESHOLD)
#         final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
#         set_motors(final_left_pwm, final_right_pwm)

#         if ramp_left_pwm != 0:
#             print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

#         time.sleep(0.01)



# def pid_control():
#     # Only applies for forward/backward, not turning
#     global left_pwm, right_pwm, left_count, right_count
#     global use_PID, KP, KI, KD, prev_movement, current_movement
#     global pid_left_base, pid_right_base

#     integral = 0
#     last_error = 0
#     last_time = monotonic()

#     # Ramping variables & params
#     ramp_left_pwm = 0
#     ramp_right_pwm = 0
#     previous_left_target = 0
#     previous_right_target = 0

#     while running:
#         current_time = monotonic()
#         dt = current_time - last_time
#         last_time = current_time

#         # --- Detect current motion mode
#         prev_movement = current_movement
#         if (left_pwm > 0 and right_pwm > 0):
#             current_movement = 'forward'
#         elif (left_pwm < 0 and right_pwm < 0):
#             current_movement = 'backward'
#         elif (left_pwm == 0 and right_pwm == 0):
#             current_movement = 'stop'
#         else:
#             current_movement = 'turn'

#         # --- NEW: reset encoders when a segment just finished
#         # (forward/backward -> not straight) OR (turn -> not turn)
#         if ((prev_movement in ('forward', 'backward') and
#              current_movement not in ('forward', 'backward'))
#             or
#             (prev_movement == 'turn' and current_movement != 'turn')):
#             reset_encoder()
#             # print(f"[ENC] reset after {prev_movement} -> {current_movement}")

#         # If we *enter* straight motion, snapshot baselines for PID
#         if current_movement in ('forward', 'backward') and prev_movement != current_movement:
#             pid_left_base = left_count
#             pid_right_base = right_count
#             integral = 0
#             last_error = 0

#         # --- PID (unchanged)
#         if not use_PID:
#             target_left_pwm = left_pwm
#             target_right_pwm = right_pwm
#         else:
#             if current_movement in ('forward', 'backward'):
#                 error = (left_count - pid_left_base) - (right_count - pid_right_base)
#                 proportional = KP * error
#                 integral += KI * error * dt
#                 integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))
#                 derivative = KD * (error - last_error) / dt if dt > 0 else 0
#                 correction = proportional + integral + derivative
#                 correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
#                 last_error = error

#                 if current_movement == 'backward':
#                     correction = -correction

#                 target_left_pwm  = left_pwm  - correction
#                 target_right_pwm = right_pwm + correction
#             else:
#                 # turning or stop: keep encoders intact *during* the state;
#                 # we already reset once at the transition above.
#                 integral = 0
#                 last_error = 0
#                 target_left_pwm  = left_pwm
#                 target_right_pwm = right_pwm

#         # --- Ramping (unchanged)
#         if use_ramping and use_PID:
#             max_change_per_cycle = RAMP_RATE * dt
#             left_diff = target_left_pwm - ramp_left_pwm
#             right_diff = target_right_pwm - ramp_right_pwm
#             left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
#             right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD
#             left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
#             right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

#             if left_direction_change:
#                 ramp_left_pwm = target_left_pwm
#             if right_direction_change:
#                 ramp_right_pwm = target_right_pwm

#             if not left_direction_change and not right_direction_change:
#                 if left_needs_ramp or right_needs_ramp:
#                     if abs(left_diff) <= max_change_per_cycle:
#                         ramp_left_pwm = target_left_pwm
#                     else:
#                         ramp_left_pwm += max_change_per_cycle if left_diff > 0 else -max_change_per_cycle

#                     if abs(right_diff) <= max_change_per_cycle:
#                         ramp_right_pwm = target_right_pwm
#                     else:
#                         ramp_right_pwm += max_change_per_cycle if right_diff > 0 else -max_change_per_cycle
#                 else:
#                     ramp_left_pwm = target_left_pwm
#                     ramp_right_pwm = target_right_pwm

#             previous_left_target = target_left_pwm
#             previous_right_target = target_right_pwm
#         else:
#             ramp_left_pwm = target_left_pwm
#             ramp_right_pwm = target_right_pwm

#         final_left_pwm  = apply_min_threshold(ramp_left_pwm,  MIN_PWM_THRESHOLD)
#         final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
#         set_motors(final_left_pwm, final_right_pwm)

#         if ramp_left_pwm != 0:
#             print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

#         time.sleep(0.01)



def pid_control():
    # Only applies for forward/backward, not turning
    global left_pwm, right_pwm, left_count, right_count
    global use_PID, KP, KI, KD, prev_movement, current_movement
    global pid_left_base, pid_right_base

    integral = 0
    last_error = 0
    last_time = monotonic()

    # Ramping variables & params
    ramp_left_pwm = 0
    ramp_right_pwm = 0
    previous_left_target = 0
    previous_right_target = 0

    # NEW: track when we've just come to a complete stop (to print once)
    was_moving = False

    while running:
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time

        # --- Detect current motion mode from commanded PWMs
        prev_movement = current_movement
        if (left_pwm > 0 and right_pwm > 0):
            current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0):
            current_movement = 'backward'
        elif (left_pwm == 0 and right_pwm == 0):
            current_movement = 'stop'
        else:
            current_movement = 'turn'

        # If we *enter* straight motion, snapshot baselines for PID
        if current_movement in ('forward', 'backward') and prev_movement != current_movement:
            pid_left_base = left_count
            pid_right_base = right_count
            integral = 0
            last_error = 0

        # ----- PID (unchanged logic) -----
        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm
        else:
            if current_movement in ('forward', 'backward'):
                error = (left_count - pid_left_base) - (right_count - pid_right_base)
                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
                derivative = KD * (error - last_error) / dt if dt > 0 else 0
                correction = proportional + integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                last_error = error

                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm  = left_pwm  - correction
                target_right_pwm = right_pwm + correction
            else:
                # turning/stop: keep encoders intact while in the state
                integral = 0
                last_error = 0
                target_left_pwm  = left_pwm
                target_right_pwm = right_pwm

        # ----- Ramping (unchanged) -----
        if use_ramping and use_PID:
            max_change_per_cycle = RAMP_RATE * dt
            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm

            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

            left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_target != 0 and previous_right_target != 0

            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm

            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    ramp_left_pwm = target_left_pwm  if abs(left_diff)  <= max_change_per_cycle else (ramp_left_pwm  + (max_change_per_cycle if left_diff  > 0 else -max_change_per_cycle))
                    ramp_right_pwm = target_right_pwm if abs(right_diff) <= max_change_per_cycle else (ramp_right_pwm + (max_change_per_cycle if right_diff > 0 else -max_change_per_cycle))
                else:
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm

            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        else:
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm

        # ----- Apply thresholds & drive motors -----
        final_left_pwm  = apply_min_threshold(ramp_left_pwm,  MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        # ----- PRINTS -----
        # Regular movement logging (like before)
        if ramp_left_pwm != 0:
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

        # NEW: first time we reach a complete stop → reset and print (0,0) ONCE
        if final_left_pwm == 0 and final_right_pwm == 0:
            if was_moving:
                reset_encoder()  # ensures they are exactly zero
                print("(Left Enc, Right Enc)=(0, 0)")
                was_moving = False
        else:
            was_moving = True  # any non-zero PWM means we’re moving (or turning)

        time.sleep(0.01)




def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            
            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()
    picam2.stop()


def pid_config_server():
    global use_PID, KP, KI, KD
    
    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            
            try:
                # Receive PID constants (4 floats)
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else: print("The robot is not using PID.")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            
            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()


def main():
    try:


        print("our listen?????????????????????????????????????")
        setup_gpio()
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()
        
        # Start wheel server (main thread)
        wheel_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()