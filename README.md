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
11. insmod DALI_driver.ko


dali_init - сканирование dali-устройств на шине и назначение им коротких номеров.

dali_send - утилита посылает команду на шину DALI, формат ./dali_send <команда> где команда это 3-байтовое число в шестнадцатиричной системе (6 HEX символов), первый байт это любое число от 0 до ff представляющий собой сиквенс запроса, и два остальных DALI-команда. В случае если команда подразумевает ответ, то ответ будет с этим же сиквенсом.


Пример:

./dali_send 230040 - команда с сиквенсом 0x23 прямой установки уровня яркости для устройства с номером 0 и уровнем яркости 0x40. Ответ: ответа на эту команду не предусмотрено.



./dali_send 2401A0 - команда с сиквенсом 0x24 получение текущего уровня яркости от устройства с номером 0. Ответ: 240040 - уровень якрости 0x40, второй байт не используется и равен у ответов всегда нулю.


