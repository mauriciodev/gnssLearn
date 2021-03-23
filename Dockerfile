FROM ubuntu:focal  AS build

RUN apt-get update && \
	apt-get install -y build-essential git cmake libtool pkg-config qttools5-dev qttools5-dev-tools cmake libeigen3-dev

COPY . /usr/src/myapp
WORKDIR /usr/src/myapp
RUN mkdir build && cd build && cmake .. && make


FROM ubuntu:bionic
COPY --from=build /usr/src/myapp/build ./
CMD ["./myapp"]

