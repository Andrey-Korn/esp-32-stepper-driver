#------------------------------------------------------
# ESP32 pin definitions for TMC2209 and mpu6050
#------------------------------------------------------

# UART
# UART0 is for onboard serial/USB chip
# UART1 used for TMC2209 x stepper
# UART2 used for TMC2209 y stepper
_uart0_tx = 1
_uart0_rx = 3
_uart1_tx = 9
# _uart1_tx = 10
_uart1_rx = 13
# _uart1_rx = 9
_uart2_tx = 26
# _uart2_tx = 17
_uart2_rx = 25
# _uart2_rx = 16

# TMC2209 step/dir/enable
_x_dir = 14
_x_step = 12
_x_en = 10
_x_diag = 35

_y_dir = 32
_y_step = 33
_y_en = 27
_y_diag = 34

# i2c for mpu6050
# esp32 default pins
_scl = 22
_sda = 21
