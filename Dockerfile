FROM alpine:3.7
RUN apk add --no-cache \
      git \
      python3 \
      python3-dev \
      py3-pip \
      build-base \
      docker \
      gcc \
      linux-headers \
      openjdk8 \
 && rm -rf /tmp/*
COPY . /opt/rosdiscover
RUN pip3 install --no-cache -e /opt/rosdiscover \
 && rm -rf /tmp/*
ENTRYPOINT ["rosdiscover"]
