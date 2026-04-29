# TGU_Robocore_2027

## 概述
本项目旨在为TGURM304提供一个完善可用的RM自瞄/导航框架

---

```bash
sudo apt update
sudo apt install libboost-all-dev
sudo apt install libopencv-dev
sudo apt install libaravis-dev aravis-tools
sudo apt install aravis-tools-cli
sudo apt install libusb-1.0-0-dev
```

```
sudo nano /etc/udev/rules.d/99-hikrobot.rules

SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0666"
```