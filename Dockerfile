# Please note that this dockerfile is for building the .deb files only

FROM ros:humble AS builder

WORKDIR /src
COPY . .

# Fix ARM64 build network error
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.ocf.berkeley.edu/ubuntu-ports/|g' /etc/apt/sources.list
RUN rosdep update
RUN apt-get update \
    && apt-get install -y \
    # Install debian packages build dependencies
    python3-bloom python3-rosdep fakeroot debhelper dh-python \ 
    dpkg \
    # Install dependencies
    && rosdep install --from-paths /src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# Complie the packages
WORKDIR /src/lgdxrobot_cloud_msgs
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

## Install the agent package for webots
WORKDIR /src
RUN dpkg -i *.deb

WORKDIR /src/lgdxrobot_cloud_adapter
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

# Organise the output
## Debian packages
WORKDIR /src
RUN mkdir -p /debs
RUN mv *.deb /debs

# Final stage outputs /src
FROM scratch AS export
COPY --from=builder /debs /debs