#! /bin/bash'
gnome-terminal --working-directory=/home/vijeth --command="bash -c ' cd Desktop/BotAccessScripts; . bot1.sh; cd catkin_workspace/src/fb5_torque_ctrl/scripts;$SHELL'"
gnome-terminal --working-directory=/home/vijeth --command="bash -c ' cd Desktop/BotAccessScripts; . bot2.sh; cd catkin_workspace/src/fb5_torque_ctrl/scripts;$SHELL'"
gnome-terminal --working-directory=/home/vijeth --command="bash -c ' cd Desktop/BotAccessScripts; . bot3.sh; cd catkin_workspace/src/fb5_torque_ctrl/scripts;$SHELL'"
sshpass -p raspberry ssh -t pi@192.168.0.250 'cd catkin_workspace/src/fb5_torque_ctrl/scripts; exec $SHELL'