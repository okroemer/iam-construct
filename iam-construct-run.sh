#!/usr/bin/env bash

# check that IP of control computer was passed
if [ $# -eq 0 ] || ([ $1 = "-h" ] || [ $1 = "-H" ])
    then
        echo "ERROR: enter the IP of the control computer as an argument:"
        echo "   ./iam-construct-run.sh <ip>"
        echo "You can optionally run the point-cloud to voxel publisher by adding the -v argument after the IP:"
        echo "   ./iam-construct-run.sh <ip> -v"
        exit 0
fi

control_comp_ip="$1"

launch_voxel=false
if [ $# -ge 2 ] && [ $2 = "-v" ]
    then
        launch_voxel=true
fi


# launch ROS things
gnome-terminal --geometry=100x5-0+0 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
-n "roscore" \
-t "roscore" \
-c "sleep 10" \
-n "rosbridge_server" \
-t "roslaunch rosbridge_server rosbridge_websocket.launch" \
-c "sleep 3" \
-n "web_vid_server" \
-t "rosrun web_video_server web_video_server" \
-c "sleep 3" \
-n "azure_kinect_driver" \
-t "roslaunch azure_kinect_ros_driver driver.launch" \
-c "sleep 3" \
-n "DEXTR-KerasTensorFlow" \
-t "cd ~/Prog/iam-interface/iam-bokeh-server; source ~/Prog/iamEnv/bin/activate; python dextr.py"


# Launch npm, bokeh, frankapy
gnome-terminal --geometry=100x5-0+275 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
-n "npm" \
-t "cd ~/Prog/iam-interface/web-interface/javascript/; source ~/Prog/iamEnv/bin/activate; npm start" \
-c "sleep 3" \
-n "bokeh" \
-t "cd ~/Prog/iam-interface/iam-bokeh-server; source ~/Prog/iamEnv/bin/activate; bokeh serve --allow-websocket-origin='*' bokeh_server.py" \
-n "frankapy" \
-t "cd ~/Prog/frankapy; ./bash_scripts/start_control_pc.sh -u student -i $control_comp_ip"


# Launch iam-vision, iam-domain-handler, iam-bt
gnome-terminal --geometry=100x5-0+550 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
-n "iam-vision" \
-t "cd ~/Prog/iam-interface/iam-vision; source ~/Prog/iamEnv/bin/activate; python iam_vision_server.py" \
-c "sleep 1" \
-n "iam-domain-handler" \
-t "cd ~/Prog/iam-interface/iam-domain-handler; source ~/Prog/iamEnv/bin/activate; ./examples/RunAll.sh" \
-c "sleep 1" \
-n "iam-bt" \
-t "cd ~/Prog/iam-interface/iam-bt; source ~/Prog/iamEnv/bin/activate; python examples/main_bt.py"


# launch point cloud to voxel publisher (if requested)
if $launch_voxel
    then
        gnome-terminal --geometry=100x5-0+825 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
        -n "PC->voxel publisher" \
        -t "cd ~/Prog/iam-interface/voxel-publisher; source ~/Prog/iamEnv/bin/activate; python scripts/point_cloud_to_voxel.py"
fi