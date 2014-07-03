import csv
FOLDER_NAME='/home/yisleyici/rosbuild_ws/ros_leap/leap_wam_controller/config'
FILE_NAME='/scores.csv'
f = open(FOLDER_NAME+FILE_NAME, "r+w")
lines=f.readlines()
lines=lines[:-1]

cWriter = csv.writer(f)
for line in lines:
    cWriter.writerow(line)
