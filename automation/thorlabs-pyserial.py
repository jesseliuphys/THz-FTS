import thorlabs_apt_protocol as apt
import serial, time, datetime
sys.path.append('./devices/')
from gentec import *

#------------------------------------------------------------
# User configuration
#------------------------------------------------------------
verbose = False

# Gentec and Thorlabs COM port (see Windows Device Manager)
gentecPort   = "COM4"
thorlabsPort = "COM3"

# Stage movement steps: (start, stop, step-size count)
list_x = range(0, 4000, 1) 

# File output name
fout_name = 'laser_automate_data_{0}.csv'.format(datetime.datetime.now().strftime("%d-%b-%Y_%H-%M-%S"))

#------------------------------------------------------------
# Intitialization
#------------------------------------------------------------
# Initialise and connect to Gentec detector 
print('\n--------------------------------')
print('Inititalizing Gentec detector connection via port {0}'.format(gentecPort))
res = Gentec(gentecPort,verbose=False)

#res.setRange('11') # 2mW [09=2microW, 10=20 microW, 11=200 microW, 12=2mW, 13=20mW]
res.getInfo()

# Initialise and connect to Thorlabs stage controller via COM port (see Windows Device Manager)
print('\n--------------------------------')
print('Inititalizing Thorlabs stage controller connection via port {0}'.format(thorlabsPort))
# Based on https://pypi.org/project/thorlabs-apt-protocol/ example
port = serial.Serial(thorlabsPort, 115200, rtscts=True, timeout=0.1)
port.rts = True
port.reset_input_buffer()
port.reset_output_buffer()
port.rts = False

# Counts to displacement conversion for Thorlabs Z8 encoder, units [1/mm]
counts_per_mm = 34303 

# Initial offset in mirror displacement
offset = int(7.1*counts_per_mm)

# Collect Thorlabs device metadata
port.write(apt.hw_req_info(source=1, dest=0x50))

print('Displacement offset: {0}'.format(offset))

#------------------------------------------------------------
# Move Thorlabs stage
# Readout Thorlabs stage position and Gentec detector power
# Save readings to file output
#------------------------------------------------------------
with open(fout_name, 'w') as f_out:
  # Write file header
  f_out.write('Datetime [dd-mm-YYYY hh:mm:ss.ff],Thorlabs counter,Thorlabs position [mm],Gentec power [Watts]\n')
  print('\nGentec datetime [dd-mm-YYYY hh:mm:ss.ff],Thorlabs counter,Thorlabs position [mm],Gentec power [Watts]')

  # Increment count positions to move stage
  for x in list_x:
    xi = offset + x
    if verbose: 
      print('\n{0}: move to position {1}'.format(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f"), xi))
    
    # Move device absolute position
    port.write(apt.mot_move_absolute(source=1, dest=0x50, chan_ident=1, position=xi))
    
    # Pause a little for mechanically stabilize actuator
    time.sleep(0.5)
    
    # Get device absolute position counter
    port.write(apt.mot_req_poscounter(source=1, dest=0x50, chan_ident=1))
    
    # Parse RX bytes sent by controller and print
    unpacker = apt.Unpacker(port)
    
    # Get power readings from Gentec detector after stage motor has stopped and stabilized
    myValues = res.getValues(1)

    # Retry a number of times if the value queried is invalid = -1
    nRetry = 10
    for it in range(nRetry):
      if myValues[0][4] == -1:
        myValues = res.getValues(1)
      else:
        break
    
    # Initialize output variables
    gentec_time, gentec_power, position_count, position_mm = -1, -1, -1, -1

    # Parse power readings from Gentec readout
    for val in myValues:
      gentec_time  = val[0].replace('OK', '')
      gentec_power = val[4]
      if verbose:
        print('{0}, {1}'.format(val[0],val[4]))

    # Parse position readings from Thorlabs readout
    for msg in unpacker:
      if verbose: print(msg)
      # Position reported by the position query function
      if msg[0] == 'mot_get_poscounter':
        position_count = msg[5]
        position_mm    = position_count / float(counts_per_mm) # convert to mm
        if verbose: 
          print('{0}, count {1} = {2} mm'.format(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f"), position_count, position_mm))

    # Format string to write out to file
    str_out = '{0},{1},{2},{3}'.format(gentec_time, position_count, position_mm, gentec_power)
    print(str_out)
    f_out.write(str_out+'\n')