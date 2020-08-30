FROM mcr.microsoft.com/vscode/devcontainers/universal:linux

USER root

# [Option] Install Docker CLI
ARG INSTALL_DOCKER="false"
COPY library-scripts/docker-debian.sh /tmp/library-scripts/
RUN if [ "${INSTALL_DOCKER}" = "true" ]; then \
        bash /tmp/library-scripts/docker-debian.sh "true" "/var/run/docker-host.sock" "/var/run/docker.sock" "codespace"; \
    else \
        echo '#!/bin/bash\nexec "$@"' > /usr/local/share/docker-init.sh && chmod +x /usr/local/share/docker-init.sh; \
    fi \
    && rm -rf /var/lib/apt/lists/* /tmp/library-scripts/

# Enable debian stretch backports
RUN echo "deb http://ftp.debian.org/debian stretch-backports main" > \
        /etc/apt/sources.list.d/stretch-backports.list

# Install dependencies
RUN apt-get update &&\
    apt-get -t stretch-backports -y install --no-install-recommends \
        cmake \
        &&\
    rm -rf /var/lib/apt/lists/*

USER codespace
