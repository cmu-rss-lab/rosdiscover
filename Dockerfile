FROM alpine:3.7
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
 && pip3 install --no-cache -r requirements.txt \
 && pip3 install --no-cache . \
 && rm -rf /tmp/*
ENTRYPOINT ["rosdiscover"]
