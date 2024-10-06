import serial
import time
import random
import threading
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

# Create PixelStrip object with the above configuration
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()  # Initialize the library (must be called once before other functions).

# Set up the serial connection to the LiDAR sensor (keep open for batch updates)
ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)

# Track the state of each LED (on or off based on the flicker decision)
led_states = [False] * LED_COUNT  # False means the LED is off, True means the LED is on

# Global variable to hold the current distance from the LiDAR sensor
current_distance = None

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
                else:
                    print(f"Invalid frame received: {data}")
            time.sleep(lidar_read_delay)  # Use the configurable delay between sensor reads
    except serial.SerialException as e:
        print(f"Error reading from LiDAR: {e}")

def flicker_color():
    """Generate a random flame-like color."""
    r = random.randint(180, 255)  # Red varies from bright to slightly dim
    g = random.randint(50, 150)   # Green adds warmth, varies between orange and yellow
    b = random.randint(0, 30)     # Blue is very low, giving it a warm tone
    return Color(r, g, b)

def update_leds(strip, active_leds):
    """Update the LEDs based on the current led_states and active_leds."""
    # Turn on the LEDs that should be on, up to the active LED count
    for i in range(active_leds):
        if led_states[LED_COUNT - i - 1]:  # Light up LEDs in reverse order (99 to 0)
            color = flicker_color()  # Use flicker color
            strip.setPixelColor(LED_COUNT - i - 1, color)
        else:
            strip.setPixelColor(LED_COUNT - i - 1, Color(0, 0, 0))  # LED remains off

    # Ensure all LEDs beyond the active LED count are turned off
    for i in range(active_leds, LED_COUNT):
        strip.setPixelColor(LED_COUNT - i - 1, Color(0, 0, 0))  # Turn off LEDs beyond the active range

    strip.show()

def decide_led_states(target_leds):
    """Make a one-time flicker decision for the LEDs. 90% chance to turn on, 10% to stay off."""
    # Decide for the range of LEDs that should be active (target_leds)
    for i in range(target_leds):
        # Only make a decision if the LED is currently off
        if not led_states[LED_COUNT - i - 1]:
            # 90% chance to turn the LED on, 10% chance to leave it off
            led_states[LED_COUNT - i - 1] = random.random() > 0.1

def distance_to_leds(distance):
    """Map distance to the number of active LEDs. Closer distance lights up more LEDs."""
    max_active_leds = int(LED_COUNT * 0.9)  # Limit to 90% of the total LEDs
    if distance is not None:
        if distance <= 90:  # Minimum distance (90 cm)
            return max_active_leds  # Max LEDs are lit (90% of total LEDs)
        elif distance >= 900:  # Maximum distance (900 cm)
            return 0  # No LEDs are lit
        else:
            # Scale the number of active LEDs proportionally between 90 cm and 900 cm
            return int((900 - distance) / 810 * max_active_leds)
    return 0

def fade_leds(current_leds, target_leds, duration, step_time=0.05):
    """Fade the number of active LEDs from current to target over a duration."""
    steps = int(duration / step_time)  # Number of steps for the transition
    step_size = (target_leds - current_leds) / steps  # Number of LEDs to adjust per step

    for step in range(steps):
        # Incrementally move towards the target number of LEDs
        current_leds += step_size
        active_leds = int(current_leds)  # Convert to integer for actual LED control
        
        # Update the LEDs based on the current state and active count
        update_leds(strip, active_leds)
        
        # Wait for the next step
        time.sleep(step_time)
    
    # Ensure we end exactly at the target number of LEDs
    update_leds(strip, target_leds)

def start_lidar_thread():
    """Start the LiDAR distance reading thread."""
    lidar_thread = threading.Thread(target=read_lidar_distance)
    lidar_thread.daemon = True  # Daemon thread will exit when the main program exits
    lidar_thread.start()

if __name__ == "__main__":
    try:
        print('Press Ctrl-C to quit.')
        last_distance = None
        current_leds = 0

        # Start the LiDAR sensor reading thread
        start_lidar_thread()

        while True:
            # Get the current distance safely from the LiDAR reading thread
            with distance_lock:
                distance = current_distance

            if distance is not None and distance != last_distance:
                print(f"Distance: {distance} cm")
                last_distance = distance

                # Get the target number of LEDs based on the distance
                target_leds = distance_to_leds(distance)

                # Fast turn-on if the target LEDs increase, slow turn-off if the target LEDs decrease
                if target_leds > current_leds:
                    # Make the flicker decision for new LEDs when increasing
                    decide_led_states(target_leds)
                    # Immediately activate the LEDs if increasing
                    update_leds(strip, target_leds)
                    current_leds = target_leds  # Update immediately to the new target
                elif target_leds < current_leds:
                    # Fade out over 3 seconds if decreasing
                    fade_leds(current_leds, target_leds, duration=3.0)
                    current_leds = target_leds

            # Small delay to keep the main loop running smoothly
            time.sleep(0.05)

    except KeyboardInterrupt:
        # Clear the LEDs when exiting
        strip.setBrightness(0)
        strip.show()
        ser.close()  # Close the serial connection to the LiDAR
