#!/usr/bin/env bash
gnome-terminal --geometry=100x5-0+0 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
-t "roscore" \
-c "sleep 3" \
-t "roslaunch rosbridge_server rosbridge_websocket.launch" \
-c "sleep 1" \
-n "web_vid_server" \
-t "rosrun web_video_server web_video_server" \


gnome-terminal --geometry=100x5-0+275 --hide-menubar --window-with-profile=IAMterm -- bash tabber.sh \
-n "npm" \
-t "cd iam-interface/web-interface/javascript; npm start" \
-c "sleep 3" \
-n "bokeh" \
-t "cd iam-interface/iam-bokeh-server; bokeh serve --allow-websocket-origin='*' bokeh_server.py" \


