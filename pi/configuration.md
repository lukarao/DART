# Pi Zero Configuration

Board: Raspberry Pi Zero 2 W

OS: Raspberry Pi OS Lite (32-bit)

## System Configuration

### Hostname

Hostname: `raspberrypi`

### Localization

Capital city: `Washington, D.C. (United States)`

Time zone: `America/New_York`

Keyboard layout: `us`

### User

Username: `pi`

Password: `raspberry`

### Wi-Fi

SSID: `drone-hotspot`

Password: `password`

## systemd

```
$ sudo nano /etc/systemd/system/stream-camera.service
```

```
[Unit]
Description=Stream camera
After=network.target

[Service]
User=pi
ExecStart=/usr/bin/rpicam-vid -t 0 -n --inline --width 640 --height 640 --framerate 24 --listen -o tcp://0.0.0.0:3000
Restart=always

[Install]
WantedBy=multi-user.target
```

```
$ sudo systemctl enable stream-camera.service
```
