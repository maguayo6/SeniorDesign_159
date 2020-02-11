import can
import time

bus = can.Bus(bustype = 'socketcan',channel='can0')
#msg = can.Message(arbitration_id = 1,data=[1,2,3,4,5])

while(True):
    time.sleep(0.5)
    print(bus.recv())
#    print(list(bus.recv().data))
    #print(list(b.data))
    