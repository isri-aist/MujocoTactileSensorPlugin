# Development
This is a collection of Dockerfiles for development purposes. The Dockerfiles are organized by programming language and/or framework. Each Dockerfile is designed to be used as a base image for a development environment. The Dockerfiles are designed to be used with the `docker-compose` command. The `docker-compose.yml` file is included in each directory.

## Usage
To use the Dockerfiles, you must have Docker installed on your machine. You can download Docker from the [official website](https://www.docker.com/).

To build the Docker image and run the container, use the following command:

```bash
docker-compose up -d
```

For windows users, you may need to use the following command:

```bash
docker-compose -f docker-compose.windows.yml up -d
```

To enter the container and build ROS2 packages, use the following command:

```bash
docker exec -it mujoco_ros_utils bash
colcon build --cmake-args -DMUJOCO_ROOT_DIR=/root/install/mujoco-2.3.7
```
