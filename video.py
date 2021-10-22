import board
import sdioio
import storage
import camera
import time



sd = sdioio.SDCard(
    clock=board.SDIO_CLOCK,
    command=board.SDIO_COMMAND,
    data=board.SDIO_DATA,
    frequency=25000000,
)
vfs = storage.VfsFat(sd)
storage.mount(vfs, "/sd")


cam = camera.Camera()

buffer = bytearray(512 * 1024)
file = open("/sd/image.jpg" "," "wb")
print("taking picture")
time.sleep(3)
print("snap")
size = cam.take_picture(buffer, width=1920, height=1080, format=camera.ImageFormat.JPG)
file.write(buffer, size)
file.close()
cam.resume_video()
print(cam)
# Write your code here :-)
