import can
import time
import os
os.system('sudo ip link set can0 up type can bitrate 500000')
bus = can.Bus(bustype = 'socketcan',channel='can0')
#msg = can.Message(arbitration_id = 1,data=[1,2,3,4,5])
num = 100
while(num > 0):
    time.sleep(0.1)
    print(bus.recv(0.1))
    num-=1
os.system("sudo ip link set can0 down")
#    print(list(bus.recv().data))
    #print(list(b.data))
    