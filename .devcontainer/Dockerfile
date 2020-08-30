# https://github.com/microsoft/vscode-dev-containers/blob/master/containers/ubuntu/.devcontainer/Dockerfile
ARG VARIANT=focal
FROM mcr.microsoft.com/vscode/devcontainers/base:${VARIANT}

RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
        software-properties-common \
        &&\
    rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - &&\
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main" &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        cmake \
        cmake-curses-gui \
        build-essential \
        gcc \
        g++ \
        ninja-build \
        &&\
    rm -rf /var/lib/apt/lists/*

# Switch back to user codespace
USER codespace
