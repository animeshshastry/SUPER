USER_UID := $(shell id -u)
USER_GID := $(shell id -g)

build:
	docker build \
	--build-arg USER_UID=$(USER_UID) \
	--build-arg USER_GID=$(USER_GID) \
	-f Dockerfile . -t super --rm

run:
	docker run -itd --rm --net=host --pid host --ipc host --gpus all \
    --env=\"NVIDIA_DRIVER_CAPABILITIES=all\" --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v /dev:/dev \
	-v ./../:/home/phoenix/src/ \
	--name super super

start:
	docker start super

stop:
	docker stop super

kill:
	docker kill super

attach:
	docker exec -it super bash

commit:
	docker commit super super2
	docker kill super
	docker tag super2 super
