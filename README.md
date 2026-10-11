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
