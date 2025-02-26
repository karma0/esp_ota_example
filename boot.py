# boot.py
import network, time, machine, uhashlib, urequests, os
from secrets import WIFI_SSID, WIFI_PASSWORD, GITHUB_REPO_URL, UPDATE_FILES

# Set up LED (using GPIO2 for onboard LED; adjust if needed)
led = machine.Pin(2, machine.Pin.OUT)

def set_led(state):
    # You can modify this function to produce different blink patterns for different states.
    led.value(state)

def connect_wifi():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('Connecting to WiFi...')
        sta_if.active(True)
        sta_if.connect(WIFI_SSID, WIFI_PASSWORD)
        # Blink LED quickly while trying to connect
        timeout = 10
        while not sta_if.isconnected() and timeout:
            set_led(not led.value())
            time.sleep(0.5)
            timeout -= 1
        if not sta_if.isconnected():
            print("Could not connect to WiFi. Proceeding without update check.")
        else:
            print("WiFi connected:", sta_if.ifconfig())
            # Solid LED to indicate successful connection
            set_led(1)
    return sta_if.isconnected()

def get_file_hash(filename=None, content=None):
    h = uhashlib.sha256()
    if content:
        h.update(content)
    elif filename:
        try:
            with open(filename, 'rb') as f:
                while True:
                    chunk = f.read(1024)
                    if not chunk:
                        break
                    h.update(chunk)
        except OSError:
            return None  # File does not exist
    return h.digest()

def update_file(filename):
    url = GITHUB_REPO_URL + filename
    print("Checking file:", filename)
    try:
        response = urequests.get(url)
        if response.status_code == 200:
            remote_content = response.content
            remote_hash = get_file_hash(content=remote_content)
            try:
                with open(filename, 'rb') as f:
                    local_hash = get_file_hash(filename)
            except OSError:
                local_hash = None  # File doesn't exist locally
            if local_hash != remote_hash:
                print("Updating file:", filename)
                with open(filename, 'wb') as f:
                    f.write(remote_content)
                response.close()
                return True  # Indicate an update was made
            else:
                print("File is up to date:", filename)
        else:
            print("Failed to fetch file:", filename, "HTTP", response.status_code)
    except Exception as e:
        print("Error fetching file:", filename, e)
    return False

# Main boot sequence
wifi_connected = connect_wifi()
update_needed = False
if wifi_connected:
    for file in UPDATE_FILES:
        if update_file(file):
            update_needed = True
    if update_needed:
        print("Updates installed. Restarting...")
        time.sleep(2)
        machine.reset()
    else:
        print("No updates detected. Proceeding to main execution.")
else:
    print("Skipping update check due to WiFi failure.")

# Set LED to off (or to a state that indicates transition to main.py)
set_led(0)
