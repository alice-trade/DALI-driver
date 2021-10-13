# DALI-driver

1. sudo rpi-update
2. sudo reboot
3. sudo apt-get update && apt-get upgrade
4. sudo apt install git bc bison flex libssl-dev
5. sudo wget https://raw.githubusercontent.com/RPi-Distro/rpi-source/master/rpi-source -O /usr/local/bin/rpi-source && sudo chmod +x /usr/local/bin/rpi-source && /usr/local/bin/rpi-source -q --tag-update
6. rpi-source
7. git clone https://github.com/alice-trade/DALI-driver
8. cd DALI-driver
9. make
10. make util
11. insmod DALI-driver.ko
