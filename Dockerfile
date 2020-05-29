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
COPY . /tmp/rosdiscover
RUN pip3 install --no-cache /tmp/rosdiscover \
 && rm -rf /tmp/*
RUN mkdir lib && cd lib && wget http://acme.able.cs.cmu.edu/public/rosdiscover/acme.standalone-ros.jar
ENTRYPOINT ["rosdiscover"]
