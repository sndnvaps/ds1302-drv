    use ds1302 to get the clock && calender then show it by ssd1306 oled screen

# ssd1306 use I2C1 interface

    SDA -> GP2 (On Board pin 4)
    SCL -> GP3 (On Board pin 5)

# ds1302 rtc connect with sio mode
   IO     -> GP16 (On Board pin 21)
   SCLK   -> GP17 (On Board pin 22)
   CE/RST -> GP18 (On Board pin 24)
   


# Deploy by picotool
