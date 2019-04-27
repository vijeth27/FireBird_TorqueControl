sshpass -p raspberry scp pi@192.168.0.250:~/catkin_workspace/src/fb5_torque_ctrl/scripts/data0.csv ~/Desktop/VoronoiRuns/data0.csv
sshpass -p raspberry scp pi@192.168.0.251:~/catkin_workspace/src/fb5_torque_ctrl/scripts/data1.csv ~/Desktop/VoronoiRuns/data1.csv
sshpass -p raspberry scp pi@192.168.0.252:~/catkin_workspace/src/fb5_torque_ctrl/scripts/data2.csv ~/Desktop/VoronoiRuns/data2.csv
sshpass -p raspberry scp pi@192.168.0.253:~/catkin_workspace/src/fb5_torque_ctrl/scripts/data3.csv ~/Desktop/VoronoiRuns/data3.csv
python DataPlottingVoronoi.py
