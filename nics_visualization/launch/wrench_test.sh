# Start the first launch file in the background
roslaunch nics_visualization ctrl_log.launch &
PID1=$!

# Start the second launch file in the background after 4 seconds
sleep 4
roslaunch nics_uav wrench.launch &
PID2=$!

# Wait for 10 seconds before stopping the processes
sleep 10
kill $PID1 $PID2
