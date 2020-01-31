import obd
import time

print("Starting OBD2 Reading.. ")

connection = obd.OBD()
cmd = obd.commands.RPM
cmd2 = obd.commands.SPEED
cmd3 = obd.commands.HYBRID_BATTERY_REMAINING

count = 0
while(count<100):
    response = connection.query(cmd)
    response2 = connection.query(cmd2)
#    response3 = connection.query(cmd3)
    
    print("RPM: ",response.value)
    print("Speed: ",response2.value.to("mph"))
#    print("Battery: ",response3.value)
    count += 1
    time.sleep(1)