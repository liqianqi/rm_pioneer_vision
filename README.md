# rm_pioneer_vision

## 使用 Docker 部署 (Recommended)

1. [Install Docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)

    ```bash
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) (Optional)

    ```bash
    sudo usermod -aG docker $USER
    newgrp docker 
    ```

3. [Authenticating to the Container registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry)

    ```bash
    export CR_PAT=YOUR_TOKEN
    echo $CR_PAT | docker login ghcr.io -u USERNAME --password-stdin
    ```

3. 拉取镜像

    ```bash
    docker pull ghcr.io/chenjunnn/rm_pioneer_vision:latest
    ```

4. 运行视觉代码
  
    ```bash
    docker run --name vision --privileged --network=host \
    ghcr.io/chenjunnn/rm_pioneer_vision:latest
    ```

    如果不希望容器一启动就运行程序，可以在命令的最后加上`zsh`替代默认指令，进入zsh shell

5. 使用`exec`命令进入容器

    ```bash
    docker exec -it vision zsh
    ```

6. 设置开机自启

    ```bash
    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service
    docker update --restart always vision
    ```

7. 安装 [rocker](https://github.com/osrf/rocker) 用于运行GUI程序 (Optional)

    ```bash
    sudo apt install python3-rocker
    ```

    - 启动rviz2
    ```bash
    rocker --x11 --devices /dev/dri/card0 \
    ghcr.io/chenjunnn/rm_pioneer_vision:latest rviz2
    ```