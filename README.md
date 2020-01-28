# Soft UART for ESP8266 RTOS SDK

This is a slightly modified sources of the extra component from
esp-open-rtos project to run on the [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK)

## Supported verstions

### ESP8266 RTOS SDK

* master
* 3.2


## How to use

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/devbis/esp8266-rtos-softuart.git
```

Add path to components in your project makefile, e.g:

```Makefile
PROJECT_NAME := my-esp-project

EXTRA_COMPONENT_DIRS := /home/user/myprojects/esp/esp8266-rtos-softuart/components

include $(IDF_PATH)/make/project.mk
```

## Credits

- [Ruslan V. Uss](https://github.com/UncleRus) developer of the original component for 
  esp-open-rtos SDK