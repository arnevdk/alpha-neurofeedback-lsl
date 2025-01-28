"""Example program to show how to read a multi-channel time series from LSL."""
import numpy as np
from mne.filter import filter_data
from pylsl import StreamInlet, resolve_byprop
import serial

SRATE=100 # 100Hz
BUFF_SIZE = 5*SRATE # Window size of 5s
#PROC_RATE  = 0.1*SRATE # Process every 0.1s
PROC_RATE  = 1*SRATE # Process every 0.1s
N_CHANNELS = 8

def main():
    # first resolve an EEG stream on the lab network
    print("looking for an EEG stream...")
    streams = resolve_byprop("type", "EEG")

    # create a new inlet to read from the stream
    inlet = StreamInlet(streams[0])

    # Initialize a circular buffer
    buffer = np.zeros((BUFF_SIZE,N_CHANNELS))
    buff_ptr = 0

    # Connect to the serial port for hardware control
    ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600)

    # Get the next sample from LSL and add it to the buffer
    proc_iter = 0
    while True:
        # get a new sample (you can also omit the timestamp part if you're not
        # interested in it)
        sample, timestamp = inlet.pull_sample()
        buffer[buff_ptr] = sample
        # Every couple of iterations, we've collected enough data and the
        # pipeline can be run
        if not proc_iter:
            metric = process(buffer)
            visualize(ser, metric)
        buff_ptr = (buff_ptr + 1) % BUFF_SIZE
        proc_iter =(proc_iter + 1) % PROC_RATE
  

def process(data):
    # Filter alpha range
    alpha = filter_data(data.T, SRATE, 8,12, verbose=False)
    # Filter beta range
    beta = filter_data(data.T, SRATE, 12,30, verbose=False)
    # Calculate alpha_power
    alpha_power = np.mean(np.var(alpha))
    # Calculate beta_power
    beta_power = np.mean(np.var(beta))
    # Metric is the log of the alpha/beta power ratio
    out = alpha_power/beta_power
    return np.log(out)

def visualize(ser, metric):
    # Draw a progressbar
    value = (1.5 + metric)*50
    value = min(max(0,value),50)
    set_height(ser, value)
    #print(str(int(round(value)))+"cm  [" + int(value)*'=' + "]")


def set_height(ser, height):
    if  height < 0 or  height > 100:
        raise ValueError('Height should be between 0 and 100')
    height = min(height, 20)
    ser.write(str(height).encode("utf-8"))

if __name__ == "__main__":
    main()
