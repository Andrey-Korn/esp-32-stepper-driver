#------------------------------------------------------
# ESP32 pin definitions for TMC2209 and mpu6050
#------------------------------------------------------

# UART
# UART0 is for onboard serial/USB chip
# UART1 used for TMC2209 x stepper
# UART2 used for TMC2209 y stepper
_uart0_tx = 1
_uart0_rx = 3

_uart1_tx = 17
# _uart1_tx = 10
_uart1_rx = 16
# _uart1_rx = 9

_uart2_tx = 12
# _uart2_tx = 17
_uart2_rx = 14
# _uart2_rx = 16

# TMC2209 step/dir/enable
_x_dir = 4
# _x_dir = 19
_x_step = 0
# _x_step = 18
# _x_en = 16
_x_en = 5
_x_diag = 34

_y_dir = 25
_y_step = 26
_y_en = 13
_y_diag = 39

# i2c for mpu6050
# esp32 default pins
_scl = 22
_sda = 21
