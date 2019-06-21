FROM jfloff/alpine-python:3.7
COPY . /tmp/rosdiscover
RUN cd /tmp/rosdiscover \
 && pip install --no-cache -r requirements.txt \
 && pip install --no-cache . \
 && rm -rf /tmp/*
