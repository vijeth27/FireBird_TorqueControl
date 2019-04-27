sshpass -p raspberry scp pi@192.168.0.250:~/catkin_workspace/src/fb5_torque_ctrl/scripts/data.csv ~/Desktop/data.csv
python DataPlotting.py
