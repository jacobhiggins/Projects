import os
import numpy as np
import math

HANDWASH_DATA_LOC = "./platform-tools/data/hand-wash-wadafied/"
NOT_HANDWASH_DATA_LOC = "./platform-tools/data/not-hand-wash-wadafied/"
WEKA_DATA_LOC = "./platform-tools/data/weka_data.csv"

data = []

class datum:
    def __init__(self,x_avg,x_std,y_avg,y_std,z_avg,z_std,activity):
        self.x_avg = x_avg
        self.x_std = x_std
        self.y_avg = y_avg
        self.y_std = y_std
        self.z_avg = z_avg
        self.z_std = z_std
        self.activity = activity

    def arrf_line():
        line = str(self.x_avg) + ","
        line = line + str(self.x_std) + ","
        line = line + str(self.y_avg) + ","
        line = line + str(self.y_std) + ","
        line = line + str(self.z_avg) + ","
        line = line + str(self.z_std) + ","
        line = line + self.activity
        return line


def read_data(activity):
    if activity=="hand-wash":
        loc = HANDWASH_DATA_LOC
    elif activity=="not-hand-wash":
        loc = NOT_HANDWASH_DATA_LOC
    else:
        print("please use correct activity label")
    dir_contents = os.listdir(loc)
    for file_name in dir_contents:
        with open(loc+file_name,"r") as f:
            data_lines = f.readlines()
            time_prev = 0.0
            x_data = np.array([])
            y_data = np.array([])
            z_data = np.array([])
            # time = float(data_lines[0].split(",")[0])*pow(10,-3)
            for data_line in data_lines:
                data_line = data_line.split(",")
                time = float(data_line[0])*pow(10,-3)
                x_datum = float(data_line[1])
                y_datum = float(data_line[2])
                z_datum = float(data_line[3])
                # print(time)

                x_data = np.append(x_data,x_datum)
                y_data = np.append(y_data,y_datum)
                z_data = np.append(z_data,z_datum)
                if time - time_prev > 1.0:

                    x_avg = np.average(x_data)
                    x_std = np.std(x_data)
                    y_avg = np.average(y_data)
                    y_std = np.std(y_data)
                    z_avg = np.average(z_data)
                    z_std = np.average(z_data)
                    if not (x_std < 0.00001):
                        d = datum(x_avg,x_std,y_avg,y_std,z_avg,z_std,activity)
                        data.append(d)

                    x_data = np.array([])
                    y_data = np.array([])
                    z_data = np.array([])

                    time_prev = time

def write_data():
    with open(WEKA_DATA_LOC,"w") as f:
        # print("hello")
        line = "x_avg,x_std,y_avg,y_std,z_avg,z_std,label\n"
        f.write(line)
        for datum in data:
            # print("hi")
            line = (str(datum.x_avg) + "," + 
                str(datum.x_std) + "," + 
                str(datum.y_avg) + "," + 
                str(datum.y_std) + "," +
                str(datum.z_avg) + "," +
                str(datum.z_std) + "," +
                str(datum.activity) + "\n")
            # print(line)
            f.write(line)


if __name__ == "__main__":
    activity = "hand-wash"
    read_data(activity)
    activity = "not-hand-wash"
    read_data(activity)
    write_data()