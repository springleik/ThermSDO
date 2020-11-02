#!/usr/bin/env python3
# file ThermSDO.py

import sys, can, time, datetime, string, struct, asyncio

# setup for SK Pang PiCAN2 interface adapter
# https://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/
# sudo ip link set can0 up type can bitrate 1000000
# canBus = can.interface.Bus(channel='can0', bustype='socketcan_native')

# setup for Lawicel CANUSB interface adapter
# https://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html
# sudo slcand -o -c -f -s8 /dev/ttyUSB0 slcan0
# sudo ifconfig slcan0 up
canBus = can.interface.Bus(channel='slcan0', bustype = 'socketcan_native')

# ----------------------------------------------------------
# command line arguments
print ('ThermSDO.py reads temperature from Maxim/Dallas DS1621')
print ('thermometer connected by I2C to Raspberry Pi computer.')
print ('M. Williamsen, 11/1/2020, https://github.com/springleik')
print ('Usage: python3 ThermSDO.py [arg names and values]')
print ('  arg name | arg value')
print ('  ---------|----------')
print ('  -addr    | DS1621 I2C address    (default is 0x48)')
print ('  -node    | CAN node number       (default is 43)')
print ('  -indx    | CAN SDO index         (default is 0x6000)')

# set defaults
ds1621Addr = 0x48
theNode = 43
theIndx = 0x6000

# check for user inputs on command line
args = iter(sys.argv)
print ('Running script: "{0}"\n  in Python: {1}'.format(next(args), sys.version))
for arg in args:
	if '-addr' == arg:
		ds1621Addr = int(next(args, ds1621Addr), 0)
	elif '-node' == arg:
		theNode = int(next(args, theNode), 0)
	elif '-indx' == arg:
		theIndx = int(next(args, theIndx), 0)
	else:
		print ('Unexpected argument: {0}'.format(arg))

# ----------------------------------------------------------
# i2c bus interface
# DS1621 register addresses
stopConv   = 0x22
accessTH   = 0xa1
accessTL   = 0xa2
readCount  = 0xa8
readSlope  = 0xa9
readTemp   = 0xaa
accessCfg  = 0xac
startConv  = 0xee

# enable interface in continuous mode
try:
	import smbus
	i2cBus = smbus.SMBus(bus = 1)
	cfg = i2cBus.read_byte_data(ds1621Addr, accessCfg)
	if 1 == (cfg & 0x01):
		cfg &= 0xfe		# clear one-shot bit
		print ('Writing config register: {0}'.format(hex(cfg)))
		i2cBus.write_byte_data(ds1621Addr, accessCfg, cfg)
		time.sleep(0.1)
	print ('DS1621 intialized at addr: {0}'.format(hex(ds1621Addr)))
	i2cBus.write_byte_data(ds1621Addr, startConv, 0)

except (IOError, OSError, ImportError) as e:
	i2cBus = None
	print ('Failed to initialize hardware: {0}'.format(e))
	print ('   Running in simulation mode.')

# function to read and report temperature
def getDataPoint():
	if not i2cBus:
		message = {'message':'Simulation mode enabled.'}
		print(message)
		return message

	# read standard (1/2-deg) resolution
	therm = i2cBus.read_word_data(ds1621Addr, readTemp)
	loRes  = (therm << 1) & 0x1fe
	loRes |= (therm >> 15) & 0x01
	if loRes > 255: loRes -= 512
	loRes /= 2.0

	# read high (1/16-deg) resolution
	count = i2cBus.read_byte_data(ds1621Addr, readCount)
	slope = i2cBus.read_byte_data(ds1621Addr, readSlope)
	temp = therm & 0xff
	if temp > 127: temp -= 256
	hiRes = temp - 0.25 + (slope - count) / slope

	# build data point structure
	now = datetime.datetime.now()
	point = {'loResC': loRes,
		'hiResC': hiRes,
		'hiResF': 32.0 + hiRes * 9.0 / 5.0,
		'date': now.strftime('%m/%d/%Y'),
		'time': now.strftime('%H:%M:%S')
		}
	return point

print ('Responding to SDO {0} at node {1}.'.format(hex(theIndx), theNode))

# notifier callback handler
def callback(msg):
	hiByte = (msg.arbitration_id >> 8) & 0xff
	loByte = msg.arbitration_id & 0xff
	# check channel and node address
	if (0x06 == hiByte) and (theNode == loByte):
		# check index
		command, index, subidx = struct.unpack('<BHB', msg.data[0:4])
		if theIndx == index:
			# check command
			if 0x40 != command:	# return abort code, read only access
				msg.data[0] = 0x80
				msg.data[4:8] = struct.pack('<L', 0x06010002)
			# check subindex
			elif 0 == subidx:	# return one byte, number of subindices
				msg.data[0] = 0x4f
				msg.data[4] = 0x02
			elif 1 == subidx:	# return four bytes, degrees Celsius
				msg.data[0] = 0x43
				msg.data[4:8] = struct.pack('<f', getDataPoint()['hiResC'])
			elif 2 == subidx:	# return four bytes, degrees Fahrenheit
				msg.data[0] = 0x43
				msg.data[4:8] = struct.pack('<f', getDataPoint()['hiResF'])
			else:				# return abort code, subindex not found
				msg.data[0] = 0x80
				msg.data[4:8] = struct.pack('<L', 0x06090011)
		# special case for QMap interface
		elif 0x1000 == index:
			# check command
			if 0x40 != command:	# return abort code, read only access
				msg.data[0] = 0x80
				msg.data[4:8] = struct.pack('<L', 0x06010002)
			# check subindex
			elif 0 == subidx:	# return null
				msg.data[0] = 0x43
				msg.data[4:8] = [0]*4
			else:				# return abort code, subindex not found
				msg.data[0] = 0x80
				msg.data[4:8] = struct.pack('<L', 0x06090011)
		else:
			# return abort code, unknown index
			msg.data[0] = 0x80
			msg.data[4:8] = struct.pack('<L', 0x06020000)

		# send reply to host
		msg.arbitration_id = 0x580 + theNode
		canBus.send(msg)

# implement main loop
async def main():
	can.Notifier(canBus, [callback], loop = asyncio.get_running_loop())
	while True: await asyncio.sleep(1)

# run main loop
try:
	asyncio.run(main())
except (KeyboardInterrupt):
	print('  User exit request.')


