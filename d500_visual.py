import serial
import math
import time
import os

PORT = "/dev/ttyUSB0"
BAUD = 230400
WIDTH = 81
HEIGHT = 41
MAX_RANGE = 6.0

# Keep latest measurement for each degree.
scan = [0.0] * 360
ser = serial.Serial(PORT, BAUD, timeout=1)
print("D500 LiDAR terminal visualizer")
print("Waiting for LiDAR data...")
last_draw = 0


def read_packet():
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0x54:
            continue
        second = ser.read(1)
        if not second or second[0] != 0x2C:
            continue
        rest = ser.read(45)
        if len(rest) != 45:
            continue
        return bytes([0x54, 0x2C]) + rest


def decode_packet(packet):
    start_angle = (packet[4] | (packet[5] << 8)) / 100.0
    points = []
    index = 6
    for _ in range(12):
        distance_mm = packet[index] | (packet[index + 1] << 8)
        intensity = packet[index + 2]
        points.append((distance_mm, intensity))
        index += 3
    end_angle = (packet[42] | (packet[43] << 8)) / 100.0
    # Deal with 359 -> 0 degree wrap.
    if end_angle < start_angle:
        end_angle += 360.0
    step = (end_angle - start_angle) / 11.0
    results = []
    for i in range(12):
        angle = (start_angle + step * i) % 360.0
        distance_mm, intensity = points[i]
        results.append((angle, distance_mm, intensity))
    return results


def draw():
    grid = [[" "] * WIDTH for _ in range(HEIGHT)]
    cx = WIDTH // 2
    cy = HEIGHT // 2
    grid[cy][cx] = "O"
    for radius_m in [1, 2, 3, 4, 5]:
        if radius_m > MAX_RANGE:
            continue
        radius_x = int(radius_m / MAX_RANGE * (WIDTH // 2 - 2))
        radius_y = int(radius_m / MAX_RANGE * (HEIGHT // 2 - 2))
        for deg in range(0, 360, 5):
            r = math.radians(deg)
            x = cx + int(math.sin(r) * radius_x)
            y = cy - int(math.cos(r) * radius_y)
            if 0 <= x < WIDTH and 0 <= y < HEIGHT:
                if grid[y][x] == " ":
                    grid[y][x] = "."
    valid = 0
    for angle_deg, distance_m in enumerate(scan):
        if distance_m <= 0 or distance_m > MAX_RANGE:
            continue
        valid += 1
        r = math.radians(angle_deg)
        x = cx + int(math.sin(r) * (distance_m / MAX_RANGE) * (WIDTH // 2 - 2))
        y = cy - int(math.cos(r) * (distance_m / MAX_RANGE) * (HEIGHT // 2 - 2))
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            grid[y][x] = "#"
    os.system("clear")
    print("D500 LIVE 360° LiDAR")
    print(f"Range: 0-{MAX_RANGE:.1f} m   Points: {valid}")
    print("Front = UP | O = LiDAR | # = obstacle | Ctrl+C = stop")
    print()
    for row in grid:
        print("".join(row))


try:
    while True:
        packet = read_packet()
        if packet is None:
            continue
        measurements = decode_packet(packet)
        for angle, distance_mm, intensity in measurements:
            if distance_mm == 0 or intensity == 0:
                continue
            degree = int(round(angle)) % 360
            scan[degree] = distance_mm / 1000.0
        now = time.time()
        if now - last_draw > 0.10:
            draw()
            last_draw = now
except KeyboardInterrupt:
    print("\nStopping D500 visualizer.")
finally:
    ser.close()
