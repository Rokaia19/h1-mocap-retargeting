ARG DOCKER_REPO
ARG ROS_DISTRO
ARG IMAGE_SUFFIX
FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX
ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user (handle existing GID/UID gracefully)
RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || usermod -l $USERNAME -d /home/$USERNAME -m $(getent passwd $USER_UID | cut -d: -f1) \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Install additonal packages - add any that you need
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip python-is-python3 ssh neovim git tmux
ENV SHELL /bin/bash

# Install Miniconda (as root before switching users)
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -p /opt/conda \
    && rm /tmp/miniconda.sh \
    && chown -R $USERNAME:$USERNAME /opt/conda

# Source ROS environment automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /opt/conda/etc/profile.d/conda.sh" >> /home/$USERNAME/.bashrc
# Set the default user
USER $USERNAME
CMD ["/bin/bash"]
