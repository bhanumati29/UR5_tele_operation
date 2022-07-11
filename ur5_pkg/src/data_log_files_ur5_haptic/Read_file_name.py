import os

# folder path
dir_path = r'/home/bhanu/catkin_ws/src/ur5_pkg/src/data_log_files_ur5_haptic'

# list to store files
files = []

# Iterate directory
for i in os.listdir(dir_path):
    # check if current path is a file
    if os.path.isfile(os.path.join(dir_path, i)):
        files.append(i)
        print("'"+i+"'"+", ")