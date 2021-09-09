FROM python:3.9-alpine
RUN apk add --no-cache \
      git \
      python3-dev \
      libffi-dev \
      musl-dev \
      libressl-dev \
      openssl-dev \
      cargo \
      py3-pip \
      build-base \
      docker \
      gcc \
      linux-headers \
      openjdk8 \
      bash \
 && rm -rf /tmp/*
ENV CRYPTOGRAPHY_DONT_BUILD_RUST=1
#RUN /bin/sh -c python3 -m ensurepip --upgrade
RUN CRYPTOGRAPHY_DONT_BUILD_RUST=1 pip3 install --no-binary cryptography pipenv cryptography
RUN mkdir -p /opt/rosdiscover/
WORKDIR /opt/rosdiscover/
COPY . /opt/rosdiscover/
RUN pipenv install
#ENTRYPOINT ["/bin/bash"]
ENTRYPOINT ["pipenv", "run", "rosdiscover"]
