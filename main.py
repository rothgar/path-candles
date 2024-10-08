import serial
import time
import random
import threading
import os
from flask import Flask, render_template, redirect, url_for, jsonify
from rpi_ws281x import PixelStrip, Color

# LED strip configuration:
LED_COUNT = 100        # Number of LED pixels.
LED_PIN = 18           # GPIO pin connected to the NeoPixels (must support PWM).
LED_FREQ_HZ = 800000   # LED signal frequency in hertz (usually 800kHz)
LED_DMA = 10           # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255   # Set to 0 for darkest and 255 for brightest
LED_INVERT = False     # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0        # PWM channel (0 or 1)

# LiDAR sensor configuration:
LIDAR_PORT = '/dev/ttyAMA0'  # LiDAR sensor connected to UART on /dev/ttyAMA0
LIDAR_BAUDRATE = 115200
lidar_read_delay = 0.5  # Delay between each LiDAR sensor read (in seconds)

# Global variables to hold the current distance and light status
current_distance = None
lights_enabled = True
chase_mode_enabled = False
flicker_mode_enabled = False
distance_following_enabled = False  # New mode for lighting based on LiDAR distance

# Flask app setup
app = Flask(__name__)

# Create PixelStrip object with the above configuration
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()  # Initialize the library (must be called once before other functions).

# Set up the serial connection to the LiDAR sensor
ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)

# Thread lock to safely share data between threads
distance_lock = threading.Lock()

def read_lidar_distance():
    """Continuously reads distance from the LiDAR sensor in a background thread."""
    global current_distance
    try:
        while True:
            # Check if there is enough data available in the buffer
            if ser.in_waiting >= 9:
                # Read all available data and only keep the most recent 9 bytes (1 frame)
                data = ser.read(ser.in_waiting)[-9:]
                
                if data[0] == 0x59 and data[1] == 0x59:  # Check for valid frame header
                    distance = data[2] + data[3] * 256   # Calculate distance in cm
                    with distance_lock:
                        current_distance = distance  # Update the global distance
            time.sleep(lidar_read_delay)  # Use the configurable delay between sensor reads
    except serial.SerialException as e:
        print(f"Error reading from LiDAR: {e}")

def flicker_color(brightness):
    """Generate a color with a flicker effect by varying brightness."""
    r = int(random.randint(180, 255) * brightness)  # Red varies from bright to slightly dim
    g = int(random.randint(50, 150) * brightness)   # Green adds warmth, varies between orange and yellow
    b = int(random.randint(0, 30) * brightness)     # Blue is very low, giving it a warm tone
    return Color(r, g, b)

def flame_flicker_effect():
    """Run the flame flicker effect with random LEDs flickering."""
    flicker_pixels = random.sample(range(LED_COUNT), k=int(LED_COUNT * 0.1))  # Flicker 10% of LEDs
    max_pixels = 11
    duration = 10 
    while flicker_mode_enabled:
        start_time = time.time()
        while time.time() - start_time < duration:
            for pixel in flicker_pixels:
                brightness = random.uniform(0.5, 1.0)  # Random brightness to simulate flicker
                color = flicker_color(brightness)
                strip.setPixelColor(pixel, color)
            strip.show()
            time.sleep(random.uniform(0.05, 0.09))  # Random flicker speed for each LED


        # Add a new random pixel to the flicker_pixels array if there's room
        if len(flicker_pixels) < max_pixels:
            new_pixel = random.choice([i for i in range(LED_COUNT) if i not in flicker_pixels])
            flicker_pixels.append(new_pixel)

        # Remove a pixel randomly to keep the flicker effect dynamic
        if len(flicker_pixels) >= max_pixels:
            removed_pixel = flicker_pixels.pop(random.randint(0, len(flicker_pixels) - 1))
            strip.setPixelColor(removed_pixel, Color(0, 0, 0))  # Turn off the removed pixel
            strip.show()

        # Pause before updating again
        time.sleep(random.uniform(0.05, 0.1))

    # Turn off lights when flicker mode is disabled
    turn_off_lights()

def distance_to_leds(distance):
    """Map the distance from the LiDAR sensor to the number of active LEDs."""
    max_active_leds = int(LED_COUNT * 0.9)  # Light up a maximum of 90% of the LEDs
    if distance is not None:
        if distance <= 90:  # Minimum distance (90 cm)
            return max_active_leds  # Light up the maximum number of LEDs
        elif distance >= 900:  # Maximum distance (900 cm)
            return 0  # No LEDs should be lit
        else:
            # Scale the number of active LEDs proportionally between 90 cm and 900 cm
            return int((900 - distance) / 810 * max_active_leds)
    return 0

def decide_led_states(target_leds):
    """Randomly decide which LEDs to turn on, with 90% chance to turn on."""
    led_states = [False] * LED_COUNT
    for i in range(target_leds):
        led_states[i] = random.random() > 0.1  # 90% chance to turn on each LED
    return led_states

def update_leds(strip, led_states):
    """Update the LED strip based on the current LED states."""
    for i in range(LED_COUNT):
        if led_states[i]:
            color = flicker_color(1.0)  # Full brightness for the non-flicker LEDs
            strip.setPixelColor(i, color)
        else:
            strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()

def distance_following_effect():
    """Update the LEDs based on the distance measured by the LiDAR sensor."""
    while distance_following_enabled:
        with distance_lock:
            distance = current_distance
        target_leds = distance_to_leds(distance)
        led_states = decide_led_states(target_leds)
        update_leds(strip, led_states)
        time.sleep(0.1)  # Small delay to prevent rapid updates

    # Turn off the lights when distance-following mode is disabled
    turn_off_lights()

def chase_mode():
    """Run chase mode, where LEDs chase from one end to the other and bounce back."""
    direction = 1  # 1 means forward, -1 means backward
    current_led = 0
    
    while chase_mode_enabled:
        strip.setPixelColor(current_led, flicker_color(1.0))  # Turn on the current LED
        strip.show()
        time.sleep(0.05)  # Faster speed for the chase effect
        strip.setPixelColor(current_led, Color(0, 0, 0))  # Turn off the previous LED

        # Update the LED index based on direction
        current_led += direction

        # If we reach the end (LED_COUNT-1) or beginning (0), reverse the direction
        if current_led == LED_COUNT - 1 or current_led == 0:
            direction *= -1

    # Turn off lights when chase mode is disabled
    turn_off_lights()

def turn_off_lights():
    """Turn off all the LEDs."""
    for i in range(LED_COUNT):
        strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()

def start_lidar_thread():
    """Start the LiDAR distance reading thread."""
    lidar_thread = threading.Thread(target=read_lidar_distance)
    lidar_thread.daemon = True  # Daemon thread will exit when the main program exits
    lidar_thread.start()

@app.route('/')
def index():
    """Main page displaying the current distance and controls."""
    return render_template('index.html', lights_enabled=lights_enabled, chase_mode_enabled=chase_mode_enabled, flicker_mode_enabled=flicker_mode_enabled, distance_following_enabled=distance_following_enabled)

@app.route('/toggle_lights')
def toggle_lights():
    """Toggle the state of the lights (on/off)."""
    global lights_enabled
    lights_enabled = not lights_enabled
    if lights_enabled:
        print("Lights turned on")
    else:
        print("Lights turned off")
        turn_off_lights()
    return redirect(url_for('index'))

@app.route('/toggle_chase_mode')
def toggle_chase_mode():
    """Toggle chase mode on and off."""
    global chase_mode_enabled
    chase_mode_enabled = not chase_mode_enabled
    if chase_mode_enabled:
        print("Chase mode enabled")
        threading.Thread(target=chase_mode).start()  # Run chase mode in a separate thread
    else:
        print("Chase mode disabled")
        turn_off_lights()
    return redirect(url_for('index'))

@app.route('/toggle_flicker_mode')
def toggle_flicker_mode():
    """Toggle flicker mode on and off."""
    global flicker_mode_enabled
    flicker_mode_enabled = not flicker_mode_enabled
    if flicker_mode_enabled:
        print("Flicker mode enabled")
        threading.Thread(target=flame_flicker_effect).start()  # Run flicker mode in a separate thread
    else:
        print("Flicker mode disabled")
        turn_off_lights()
    return redirect(url_for('index'))

@app.route('/toggle_distance_following')
def toggle_distance_following():
    """Toggle distance-following mode on and off."""
    global distance_following_enabled
    distance_following_enabled = not distance_following_enabled
    if distance_following_enabled:
        print("Distance following mode enabled")
        threading.Thread(target=distance_following_effect).start()  # Run distance-following in a separate thread
    else:
        print("Distance following mode disabled")
        turn_off_lights()
    return redirect(url_for('index'))

@app.route('/reboot')
def reboot():
    """Reboot the system."""
    print("Rebooting the system...")
    os.system('sudo reboot')
    return redirect(url_for('index'))

@app.route('/shutdown')
def shutdown():
    """Shut down the system."""
    print("Shutting down the system...")
    os.system('sudo shutdown now')
    return redirect(url_for('index'))

@app.route('/get_distance')
def get_distance():
    """Endpoint to get the current distance."""
    with distance_lock:
        distance = current_distance if current_distance is not None else 'No data'
    return jsonify({'distance': distance})

if __name__ == "__main__":
    # Start the LiDAR sensor reading thread
    start_lidar_thread()

    # Run the Flask web server with hot reload enabled
    app.run(host='0.0.0.0', port=5000, debug=True)
