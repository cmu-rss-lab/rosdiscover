FROM alpine:3.8
RUN apk add --no-cache \
      git \
      python3 \
      python3-dev \
      libffi-dev \
      libressl-dev \
      py3-pip \
      build-base \
      docker \
      gcc \
      linux-headers \
      openjdk8 \
      bash \
      less \
      musl-dev \
      libffi-dev \
 && rm -rf /tmp/*
COPY . /opt/rosdiscover
RUN pip3 install -r /opt/rosdiscover/requirements.dev.txt && rm -f /tmp/*
RUN pip3 install --no-cache -e /opt/rosdiscover \
 && rm -rf /tmp/*
ENTRYPOINT ["rosdiscover"]
