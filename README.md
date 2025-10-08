A simple DKMS driver for WorldSemi WS2812B LEDs to be controlled via
SPI. So far only tested on an RK3588s based Indiedroid Nova with an
array of 4 lights and a Raspberry Pi 5 with an array of 8 lights.

Note: there is a known issue where the first LED in the array does
not light up. I am unsure at this time why this occurs. I am open
to suggestions or pull requests if you can solve this issue.
