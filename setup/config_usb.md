## Kiểm tra thiết bị
```
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
```

## Tạo rule
```
sudo nano /etc/udev/rules.d/99-usb-devices.rules
```

### Nội dung file
```
# USB_CAN (driver)
SUBSYSTEM=="tty", KERNELS=="1-2.2:1.0", SYMLINK+="usbcan", MODE="0666"

# ESP32
SUBSYSTEM=="tty", KERNELS=="1-2.4.1.3:1.0", SYMLINK+="esp32", MODE="0666"

# Lidar
SUBSYSTEM=="tty", KERNELS=="1-2.1:1.0", SYMLINK+="rplidar", MODE="0666"

# Battery capacity
SUBSYSTEM=="tty", KERNELS=="1-2.4.2:1.0", SYMLINK+="battery", MODE="0666"

# Magnetic sensor
SUBSYSTEM=="tty", KERNELS=="1-2.4.3:1.0", SYMLINK+="magnetic", MODE="0666"
```

## Kích hoạt rule
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

# Kiểm tra
```
ls -l /dev | grep usbcan
ls -l /dev | grep esp32
ls -l /dev | grep rplidar
ls -l /dev | grep battery
ls -l /dev | grep magnetic
```

