BASE_DIR=$(realpath "$(dirname $0)")

rm -r ${BASE_DIR}/tj2_interfaces/msg
rm -r ${BASE_DIR}/tj2_interfaces/srv
sudo rm -r ${BASE_DIR}/tj2_interfaces/tj2_interfaces
sudo rm -r ${BASE_DIR}/tj2_interfaces/.gradle
cp -r ~/tj2_ros/src/tj2_interfaces/msg ${BASE_DIR}/tj2_interfaces/msg
cp -r ~/tj2_ros/src/tj2_interfaces/srv ${BASE_DIR}/tj2_interfaces/srv

python << EOF
import os
base_dir = "${BASE_DIR}/tj2_interfaces/"
messages = os.listdir(base_dir + "/msg")
services = os.listdir(base_dir + "/srv")

with open(base_dir + "CMakeListsTemplate.txt") as file:
    contents = file.read()
contents = contents.replace("MESSAGE_LIST", "\n  ".join(messages))
contents = contents.replace("SERVICE_LIST", "\n  ".join(services))

with open(base_dir + "CMakeLists.txt", 'w') as file:
    file.write(contents)
EOF

docker build -f ./Dockerfile -t diffyjr_ros_genjava:latest .

docker run -it --rm \
    -v ${BASE_DIR}/../libs:/artifacts:rw \
    -v ${BASE_DIR}/tj2_interfaces:/root/catkin_ws/src/tj2_interfaces:rw \
    diffyjr_ros_genjava:latest \
    /bin/bash -c "source /root/rosjava/devel/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make -j4 && \
    cp /root/catkin_ws/build/tj2_interfaces/java/tj2_interfaces/build/libs/tj2_interfaces-0.0.0.jar /artifacts/tj2_interfaces_msgs.jar"
