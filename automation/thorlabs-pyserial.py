'''

Welcome to thorlabs-pyserial.py

This script operates the BREAD spectrometer in Lab MCP 011:
- moves the mirror on the motorized stage by regular steps,
- queries the actual moved position, 
- reads the Gentec detector power,
- does basic plotting and periodogram of the data

Configure various settings under "User configuration" inside script

Run by typing:
  python thorlabs-pyserial.py

'''

import thorlabs_apt_protocol as apt
import serial, time, datetime, math
import matplotlib.pyplot as plt
import numpy as np
from gentec import *
from scipy import signal

#------------------------------------------------------------
# User configuration: global settings (unchanged btwn runs)
#------------------------------------------------------------
# Thorlabs Z8-812 stage encoder: counts to displacement conversion  
counts_per_mm = 34303 # units [1/mm]

verbose       = False
realtime_plot = False # plot power vs position in realtime
save_end_plot = True # output pdf plot at end of scan

# Gentec and Thorlabs COM port (see Windows Device Manager)
gentecPort   = "COM4"
thorlabsPort = "COM3"

# Time to pause between each step of stage
sleep_btwn_steps = 0.1

#------------------------------------------------------------
# User configuration: per run settings
#------------------------------------------------------------
# Stage movement steps: (start, stop, step-size count)
stop_step = 300000 # encoder counts
step_size = 2 # encoder counts
list_x    = range(0, stop_step, step_size) 

# The zero offset in mm after homeing stage
offset_in_mm = 2.5

# File output name
#fout_name = 'IR-Si253_10V_1p5A_totalsteps60000_stepsize10_automate_data_{0}.csv'.format(datetime.datetime.now().strftime("%d-%b-%Y_%H-%M-%S"))
fout_name = 'laser_automate_data_{0}_nstep{1:.0f}_stepsize{2}.csv'.format(datetime.datetime.now().strftime("%d-%b-%Y_%H-%M-%S"), stop_step/float(step_size), step_size)

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

# Collect Thorlabs device metadata
port.write(apt.hw_req_info(source=1, dest=0x50))

# Initial offset in mirror displacement
offset = int( offset_in_mm * counts_per_mm )
print('Displacement zero offset: {0}'.format(offset))

# Store list of values for realtime plotting
pval = []
xval = []
yval = []

# Initialize matplotlib
fig, (ax1, ax2) = plt.subplots(2)

#------------------------------------------------------------
# Move Thorlabs stage
# Readout Thorlabs stage position and Gentec detector power
# Save readings to file output
#------------------------------------------------------------


with open(fout_name, 'w') as f_out:
  # Write file header
  f_out.write('Datetime [dd-mm-YYYY hh:mm:ss.ff],Thorlabs counter,Thorlabs position [mm],Gentec power [Watts]\n')
  print('\nCount,Gentec datetime [dd-mm-YYYY hh:mm:ss.ff],Thorlabs counter,Thorlabs position [mm],Gentec power [Watts]')

  # Increment count positions to move stage
  for i, x in enumerate(list_x):
    xi = offset + x
    if verbose: 
      print('\n{0}: move to position {1}'.format(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S.%f"), xi))
    
    # Move device absolute position
    port.write(apt.mot_move_absolute(source=1, dest=0x50, chan_ident=1, position=xi))
    
    # Pause a little for mechanically stabilize actuator
    time.sleep(sleep_btwn_steps)
    
    # Get device absolute position counter
    port.write(apt.mot_req_poscounter(source=1, dest=0x50, chan_ident=1))
    
    # Parse RX bytes sent by controller and print
    unpacker = apt.Unpacker(port)
    
    # Get power readings from Gentec detector after stage motor has stopped and stabilized
    myValues = res.getValues(1)
    
    # Retry a number of times if the value queried is invalid = -1
    nRetry = 5
    for it in range(nRetry):
      time.sleep(0.02)
      if myValues[0][4] <= 0.:
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
    f_out.write(str_out+'\n')
    
    # Track progress in a counter
    progress = '{0}/{1}'.format(i, len(list_x))
    # Simulation with ideal sinsusoid
    #kx = 15000*(position_mm-offset_in_mm)
    #sin_power = 1e-4+5e-5*np.sin(kx)
    #print(progress + ',' + str_out + ',{0:.5g},{1:.5g}'.format(kx,sin_power))
    print(progress + ',' + str_out)

    pval.append(position_count)
    # Append positions and powers to a list
    xval.append(position_mm)
    #yval.append(sin_power)
    yval.append(gentec_power)

    # Use matplotlib to visualize data in real time
    if realtime_plot and i > 1:

      # Plot the power vs mirror position
      ax1.plot(xval, yval, color='tab:blue')
      ax1.set(xlabel='Mirror position [mm]', ylabel='Power [Watts]')

      # Set colours of psd lines (it plots one per step right now)
      if i < 100:
        mycolor=(0.3, 0.3, 0.3, 0.1)
      elif i >= 100 and i < 200:
        mycolor='#1d91c040'
      else:
        mycolor='#ec701460'

      # sample frequency = (1/2) * (speed of light / mirror step size)
      fs = 1e4/(2.*step_size) # THz
      # Use matplotlib to compute power spectra density on lower plot
      #f, Pxx = ax2.psd(yval, Fs=fs, color=mycolor) # matplotlib psd
      f, Pxx = signal.periodogram(yval, fs=fs, window='parzen') # scipy psd
      ax2.plot(f, np.sqrt(Pxx), color=mycolor)

      # Beautify lower plot
      #ax2.set_ylim(-170, -100)
      ax2.set_xlim(0, 800)
      ax2.grid(False)
      ax2.set(xlabel='Frequency [THz]', ylabel='Power spectral density [$W$ Hz$^{-1/2}$]')
      plt.tight_layout()

      # Keep updating plot during each step
      plt.pause(0.02)

# Find the difference between consecutive steps of the stage
dx = np.diff(pval)
# Filter the differences so it is not too much more than the step size
dxfilt = [x for x in dx if x < step_size+10]

# Save figure at the end after finishing data-taking
if save_end_plot:

  # Prepare the figure and subplots
  fig0, (ax0a, ax0b, ax0c) = plt.subplots(3)
  fig0.set_size_inches(7, 10)

  # Subplot 1: plot the power vs mirror position
  ax0a.plot(xval, yval, color='tab:blue')
  ax0a.set(xlabel='Mirror position [mm]', ylabel='Power [Watts]')

  # Subplot 2: plot the final Fourier transformed psd
  fs = 1e4/(2.*step_size) # THz
  f, Pxx = signal.periodogram(yval, fs=fs, window='parzen') # scipy psd
  ax0b.plot(f, np.sqrt(Pxx), color='tab:orange')
  ax0b.set_xlim(0, 800)
  ax0b.grid(False)
  ax0b.set(xlabel='Frequency [THz]', ylabel='Power spectral density [W Hz$^{-1/2}$]')

  # Subplot 3: histogram the steps performed by the motorized stage
  ax0c.hist(dxfilt, bins=10)
  ax0c.set_xlim(0, 10)
  ax0c.set(xlabel='Motor step size', ylabel='Steps')
  plt.tight_layout()

  # Save plots to file
  fig0.savefig(fout_name.replace('.csv', '.pdf'))

plt.show()