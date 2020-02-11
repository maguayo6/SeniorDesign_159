import obd
import time

print("Starting OBD2 Reading.. ")
connection = obd.OBD()

#cmd = obd.commands.RPM
#cmd2 = obd.commands.SPEED
#cmd3 = obd.commands.ENGINE_LOAD

carCommands = [obd.commands.ENGINE_LOAD,
#               obd.commands.COOLANT_TEMP,
#               obd.commands.FUEL_PRESSURE,
               obd.commands.RPM,
               obd.commands.SPEED,
#               obd.commands.INTAKE_TEMP,
#               obd.commands.MAF,
               obd.commands.THROTTLE_POS]

count = 0
while(count<100):
    if(connection.is_connected()):
        respCommands = []
        for c in carCommands:
            respCommands.append(str(connection.query(c).value))
        print(respCommands)
#    response = connection.query(cmd)
#    response2 = connection.query(cmd2)
#    response3 = connection.query(cmd3)
    
#    print("RPM: ",response.value)
#    print("Speed: ",response2.value.to("mph"))
#    print("Battery: ",response3.value)
    count += 1
    time.sleep(1)