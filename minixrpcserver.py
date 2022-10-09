#/usr/bin/python3

from xmlrpc.server import SimpleXMLRPCServer

from minix import MiniX

minix = MiniX()

def power_on(): minix.power_on()
def power_off(): minix.power_off()
def get_voltage(): return minix.voltage
def measure_voltage(): return minix.measured_voltage
def get_current(): return minix.current
def measure_current(): return minix.measured_current
def set_voltage(v): minix.voltage = v
def set_current(c): minix.current = c
def is_enabled(): return minix.enabled

server = SimpleXMLRPCServer(("0.0.0.0", 8000), allow_none=True)
print("Listening on port 8000...")

server.register_function(power_on, "power_on")
server.register_function(power_off, "power_off")
server.register_function(get_voltage, "get_voltage")
server.register_function(set_voltage, "set_voltage")
server.register_function(measure_voltage, "measure_voltage")
server.register_function(get_current, "get_current")
server.register_function(set_current, "set_current")
server.register_function(measure_current, "measure_current")
server.register_function(is_enabled, "is_enabled")

server.serve_forever()
