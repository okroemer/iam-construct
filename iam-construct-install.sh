#!/usr/bin/env bash

BASE_FOLDER=~/Prog
SETUP_BASICS=0
SETUP_ROS_M=0
SETUP_FRANKA=0
SETUP_HUMAN=0
SETUP_SENSOR=0
SETUP_CORE=0


while getopts ":abrfhscd:" opt; do
 case $opt in
	a)	echo "Adding All (Full) Setup to Tasks" >&2
		SETUP_BASICS=1; SETUP_ROS_M=1; SETUP_FRANKA=1; SETUP_HUMAN=1; SETUP_SENSOR=1; SETUP_CORE=1; ;;
	b)	echo "Adding Basics Setup to Tasks" >&2
		SETUP_BASICS=1; ;;
	r)	echo "Adding ROS Melodic Setup to Tasks" >&2
		SETUP_ROS_M=1; ;;
	f)	echo "Adding Frankapy Setup to Tasks!" >&2
		SETUP_FRANKA=1; ;;
	h)	echo "Adding Human Interface Setup to Tasks" >&2
		SETUP_HUMAN=1; ;;
	s)	echo "Adding Sensor Setup to Tasks" >&2
		SETUP_SENSOR=1; ;;
	c)	echo "Adding Core Setup to Tasks" >&2
		SETUP_CORE=1; ;;
	d)	echo "Setting base directory to $OPTARG" >&2
		BASE_FOLDER=$OPTARG; ;;
	\?)	echo "Invalid option: -$OPTARG" >&2; ;;
  esac
done

echo -e "Installing in folder: " $BASE_FOLDER "\n"

if [ ! -d "$BASE_FOLDER/" ]; then
	mkdir $BASE_FOLDER
fi


if [[ $SETUP_BASICS -eq 1 ]]; then
	echo "INSTALLING - IAM Basics"
	cd $BASE_FOLDER
	sudo apt update

	sudo apt install -y python3-distutils
	sudo apt install -y curl 

	curl https://bootstrap.pypa.io/pip/3.6/get-pip.py | sudo -H python3.6
	sudo -H pip3.6 install numpy matplotlib virtualenv
	virtualenv -p python3.6 iamEnv
fi

if [[ $SETUP_ROS_M -eq 1 ]]; then

	echo "INSTALLING - ROS Melodic"
	cd $BASE_FOLDER

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update
	sudo apt install -y ros-melodic-desktop-full
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc

	sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo apt install -y python-rosdep
	sudo rosdep init
	rosdep update

	sudo apt install -y ros-melodic-rosbridge-server
	sudo apt install -y ros-melodic-web-video-server
fi

if [[ $SETUP_FRANKA -eq 1 ]]; then
	echo "INSTALLING - IAM Franka Interface"
	cd $BASE_FOLDER
	sudo apt update

	git clone --recurse-submodules git@github.com:iamlab-cmu/frankapy.git
	sudo apt install -y ros-melodic-libfranka ros-melodic-franka-ros

	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate --extend

	cd $BASE_FOLDER/frankapy
	pip install -e .

	source /opt/ros/melodic/setup.bash
	./bash_scripts/make_catkin.sh

	echo "source $BASE_FOLDER/frankapy/catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate

	sudo apt-get install autoconf automake libtool curl make g++ unzip
	wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.4/protobuf-all-3.11.4.zip
	unzip protobuf-all-3.11.4.zip
	cd protobuf-3.11.4
	./configure
	
	make -j4
	sudo make install
	sudo ldconfig
	cd ..
	./bash_scripts/make_proto.sh
	
fi



if [ ! -d "$BASE_FOLDER/iam-interface" ]; then
	cd $BASE_FOLDER
	git clone --recurse-submodules git@github.com:iamlab-cmu/iam-interface.git
	source $BASE_FOLDER/iamEnv/bin/activate
fi



if [[ $SETUP_HUMAN -eq 1 ]]; then
	echo "INSTALLING - IAM Human Interface"
	cd $BASE_FOLDER
	sudo apt update

	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate
	deactivate
	curl -fsSL https://deb.nodesource.com/setup_current.x | sudo -E bash -
	sudo apt-get install -y nodejs

	source $BASE_FOLDER/iamEnv/bin/activate
	pip install open3d
	pip install bokeh

	cd $BASE_FOLDER/iam-interface/web-interface/javascript
	npm install

	#let host_name = "localhost"

	cd $BASE_FOLDER/iam-interface/
	git clone https://github.com/iamlab-cmu/DEXTR-KerasTensorflow.git
	cd DEXTR-KerasTensorflow
	pip install matplotlib opencv-python pillow scikit-learn scikit-image h5py tensorflow keras

	cd models/
	chmod +x download_dextr_model.sh
	./download_dextr_model.sh
fi

if [[ $SETUP_SENSOR -eq 1 ]]; then
	echo "INSTALLING - IAM Sensor Interface"
	cd $BASE_FOLDER/iam-interface
	sudo apt update

	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate
	pip install open3d

	cd $BASE_FOLDER/iam-interface
	curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
	sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
	sudo apt-get update
	sudo apt install -y libk4a1.4 libk4a1.4-dev k4a-tools

	cd catkin_ws/src
	git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
	git clone https://github.com/ros-perception/vision_opencv.git
	cd vision_opencv
	git checkout melodic
	cd ../..
	source /opt/ros/melodic/setup.bash
	catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

	sudo mv /opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge /opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge_2.7

	cd /etc/udev/rules.d/
	sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules

	echo "source $BASE_FOLDER/iam-interface/catkin_ws/devel/setup.bash --extend" >> ~/.bashrc
	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate

	cd $BASE_FOLDER/iam-interface/perception
	pip install -e .

	cd $BASE_FOLDER/iam-interface/camera-calibration
	sudo apt install -y python3-tk python3-empy
	#pip install -e .
fi



if [[ $SETUP_CORE -eq 1 ]]; then
	echo "INSTALLING - IAM Core Interface"
	source ~/.bashrc
	source $BASE_FOLDER/iamEnv/bin/activate

	#Pillar-State
	cd $BASE_FOLDER/iam-interface	
	sudo rm -r pillar-state
	git clone --recursive git@github.com:iamlab-cmu/pillar-state.git
	cd pillar-state
	./make_scripts/make_proto_cpp.sh
	./make_scripts/make_view_cpp.sh
	pip install -e python

	#Pillar-Skills
	cd $BASE_FOLDER/iam-interface/pillar-skills
	pip install -e .

	#IAM-Skills
	cd $BASE_FOLDER/iam-interface/iam-skills
	pip install -e .

	#IAM-Domain-Handler
	cd $BASE_FOLDER/iam-interface/iam-domain-handler
	pip install -e .

	#IAM-Behavior Tree
	cd $BASE_FOLDER/iam-interface/iam-bt
	pip install -e .

fi
