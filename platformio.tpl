#
# Template for webbuilder
#
[platformio]
include_dir = Inc
src_dir = Src

[common]
build_flags =
  -I .
  -I boards
  -I FatFS
  -I Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
  -I Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  -I USB_DEVICE/Target
  -D OVERRIDE_MY_MACHINE
lib_deps =
  boards
  bluetooth
  grbl
  keypad
  motors
  trinamic
  odometer
  FatFs
  sdcard
  Core
  Class
  App
  Target
lib_extra_dirs =
  .
  FatFs
  Middlewares/ST/STM32_USB_Device_Library
  USB_DEVICE

[env]
platform = ststm32
framework = stm32cube
# Do not produce .a files for lib deps (which would prevent them from overriding weak symbols)
lib_archive = no
lib_ldf_mode = off

[env:%env_name%]
board = %board%
board_build.ldscript = %ldscript%
build_flags = ${common.build_flags}
%build_flags%
lib_deps = ${common.lib_deps}
  eeprom
%lib_deps%
lib_extra_dirs = ${common.lib_extra_dirs}
