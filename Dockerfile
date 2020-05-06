FROM alpine:3.7
RUN apk update && \
    apk fetch openjdk8 && \
	apk add openjdk8

COPY . /tmp/rosdiscover

RUN cd /tmp/rosdiscover \
 && apk add --no-cache \
      git \
      python3 \
      python3-dev \
      py3-pip \
      build-base \
      docker \
      gcc \
      linux-headers \
 && pip3 install --no-cache . \
 && rm -rf /tmp/*
RUN mkdir lib && cd lib && wget http://acme.able.cs.cmu.edu/public/rosdiscover/acme.standalone-ros.jar && cd ..
ENTRYPOINT ["rosdiscover"]
