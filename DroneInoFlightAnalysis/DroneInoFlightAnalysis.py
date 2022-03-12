"""
    @file DroneInoFlightAnalysis.py
    @author Sebastiano Cocchi
    @brief Compute data analysis for DroneIno data flight.

    Run DroneInoFlightAnalysis.py python script and choose a flight data file produced by DroneIno.
    This will show a pitch and roll data for PID fine tuning.
    
"""

from tkinter import filedialog as fd
import plotly.graph_objects as go
import csv

# # path to folder
# dirName = "data"

# # create the list of files in the dir
# fileList = os.listdir(dirName)


filename = fd.askopenfilename()

# create headers
headers = []
index = []
pitch = []
roll = []
altitude = []
battery = []

# if file is csv
if filename[-3:] == "csv":

    # open the file
    with open(filename, encoding = "cp437", errors = 'remove') as csvfile:

            # read csv
            spamreader = csv.reader(csvfile, delimiter=';')

            tmp = 0
            first_line = True

            # read row by row
            for row in spamreader:
                
                # if first line then it is header
                if (first_line) == True:
                    headers = row                                                        # roll; pitch; flightMode; battery; altitude
                    first_line = False
                    
                # fill arrays
                else:
                    row = [float(x) for x in row]
                    # if there is no reading, i.e. only 1 or 0 there's no plot
                    if (row[0] != 1 and row[1] != 1 and row[3] != 1 and row[4] != 1):
                        index.append(tmp)
                        roll.append(row[0])
                        pitch.append(row[1])
                        battery.append(row[3])
                        altitude.append(row[4])
                    
                tmp = tmp + 1
            
            # plot data
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=index, y = roll, mode="markers+lines", name = "roll"))
            fig.add_trace(go.Scatter(x=index, y = pitch, mode="markers+lines", name = "pitch"))
            # go.add_trace(index, battery, title = "battery")
            # go.add_trace(index, altitude, title = "altitude")
            
            fig.update_layout(title=filename)

            # axis
            fig.show()

else:
    print("File is not .csv")
