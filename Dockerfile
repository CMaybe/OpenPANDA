FROM ghcr.io/cmaybe/dev-panda:latest

RUN rm /bin/sh && ln -s /bin/bash /bin/sh


ARG PROJECT_NAME=openPANDA
ENV USER_NAME=panda
ENV GROUP_NAME=cau


# Add user info
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME} \
    && useradd --create-home --shell /bin/bash \
               --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	# Possible security risk
	&& echo "${USER_NAME}:${GROUP_NAME}" | sudo chpasswd \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"



# Make workspace 
ENV HOME /home/${USER_NAME}
ENV WORKSPACE ${HOME}/${PROJECT_NAME}
RUN mkdir -p ${WORKSPACE} \
	&& chown -R ${USER_NAME}:${GROUP_NAME} ${WORKSPACE}

# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"

RUN echo "export USER=${USER_NAME}" >> ${HOME}/.bashrc \
	&& echo "export GROUP=${GROUP_NAME}" >> ${HOME}/.bashrc