# Sae-Aerothon

## D500 LiDAR terminal visualizer

`d500_visual.py` displays live 360° LiDAR measurements in the terminal, with a maximum displayed range of 6 meters. It uses `/dev/ttyUSB0` at 230400 baud. Adjust `PORT` in the script if your adapter uses another device path.

### Run on Raspberry Pi

Connect the D500 serial adapter to the Pi, then install the Python serial dependency and download the script into your home directory:

```bash
sudo apt update
sudo apt install -y python3-serial curl
curl -fL https://raw.githubusercontent.com/Avya18/abcd/main/d500_visual.py -o ~/d500_visual.py
```

Run:

```bash
python3 ~/d500_visual.py
```

The display refreshes about 10 times per second. Front is up, `O` marks the LiDAR, `#` marks a measurement, and dotted rings indicate 1–5 meters. Press Ctrl+C to stop.

If opening the serial port reports permission denied, add your user to the serial access group, then log out and log back in:

```bash
sudo usermod -aG dialout "$USER"
```

## QR centering mission on Raspberry Pi 5

[qr_2.0.py](qr_2.0.py) uses a downward Picamera2 camera and a Pixhawk running ArduCopter. With `--run`, it enters GUIDED, arms, takes off to 2 m, moves forward 1 m, centers on a fully decoded QR code, saves a photo, and sends zero velocity commands indefinitely.

### Install and download

Use Raspberry Pi OS with a Picamera2-compatible camera. Install the camera, OpenCV and NumPy packages through apt, then install pymavlink in a virtual environment that can access the system camera packages:

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-venv python3-pip curl
python3 -m venv --system-site-packages ~/qr-venv
source ~/qr-venv/bin/activate
python3 -m pip install pymavlink
curl -fL https://raw.githubusercontent.com/Avya18/abcd/main/qr_2.0.py -o ~/qr_2.0.py
chmod +x ~/qr_2.0.py
python3 -m py_compile ~/qr_2.0.py
```

The virtual environment follows [Raspberry Pi's Python guidance](https://www.raspberrypi.com/documentation/usage/raspberry-pi.html) and [Picamera2 installation guidance](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf). MAVLink dependency: [pymavlink](https://github.com/ArduPilot/pymavlink).

Activate the environment in every new terminal before running the script:

```bash
source ~/qr-venv/bin/activate
python3 ~/qr_2.0.py --help
```

### Camera and QR test (no Pixhawk connection or arming)

Place a QR code under the downward camera and manually center it. A fully decoded QR must remain inside the centering tolerance for 1 second before a photo is saved:

```bash
source ~/qr-venv/bin/activate
python3 ~/qr_2.0.py --vision-test --qr-timeout 120
```

On the Pi desktop, add `--display` for a camera window. Omit it in an SSH session without a graphical display:

```bash
python3 ~/qr_2.0.py --vision-test --display --qr-timeout 120
```

If the mounted camera orientation differs, use `--rotation 90`, `--rotation 180`, `--rotation 270`, `--flip-horizontal`, or `--flip-vertical` as appropriate. Confirm the image orientation matches the forward/right centering directions before using the mission.

### Preflight only

Connect the Pixhawk. The defaults are `/dev/ttyACM0`, 921600 baud and camera index 0. This command checks the camera and telemetry; it does not command a mode change, arming, takeoff or movement:

```bash
source ~/qr-venv/bin/activate
python3 ~/qr_2.0.py --connection /dev/ttyACM0 --baud 921600
```

Use the actual serial device and configured baud rate if they differ. The script requires fresh local position, relative altitude, attitude and heartbeat messages. By default, it also requires GPS fix type at least 3 and at least 6 satellites.

If the serial device reports permission denied:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in after changing group membership.

### Run the flight mission

The following command enables arming, takeoff and movement:

```bash
source ~/qr-venv/bin/activate
python3 ~/qr_2.0.py --connection /dev/ttyACM0 --baud 921600 --run
```

The default mission is takeoff 2 m, forward 1 m, QR centering, save photo, then hover. To state the distances explicitly:

```bash
python3 ~/qr_2.0.py --connection /dev/ttyACM0 --baud 921600 --takeoff-alt 2.0 --forward-distance 1.0 --run
```

Photos are saved to `~/qr_captures/qr_lock_<timestamp>.jpg` when the script is downloaded as shown. Set another directory with `--capture-dir ~/my_qr_photos`.

The script moves toward fully decoded QR codes only; the edge-density fallback is displayed for debugging and is not accepted as a centering lock. A QR timeout exits the script. Changing flight mode with RC/QGC releases mission control. Ctrl+C stops the script and briefly sends zero velocity commands; it does **not** command landing or disarming.

Validation performed for this upload: Python syntax compilation only. Camera, Pixhawk integration and flight behavior have not been tested on hardware.
