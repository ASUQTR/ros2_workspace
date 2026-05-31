ENV="$(pwd)/env/desktop"

cp ./requirements_desktop.txt $ENV
cp ./underlay.repos $ENV

docker build -t ros2-humble -f $ENV/Dockerfile $ENV