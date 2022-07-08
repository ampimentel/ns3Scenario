FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install wget -y \
        # minimal C++ requeriments
        g++ python3 cmake \
        # minimal requirements for Python API users git  python3-setuptools
        python3-dev pkg-config sqlite3 git \
        # Netanim animator
        # qt5-default \
	# Support for bake build tool
        # autoconf cvs bzr \
        # Debugging
        gdb valgrind \
        # Documentation
        # uncrustify doxygen graphviz imagemagick \
        # texlive texlive-extra-utils texlive-latex-extra \
        # python3-sphinx dia \
        # read pcap packet
        tcpdump \
        # sqlite libsqlite3-dev \
        # Xml-based version of the config store
        libxml2 libxml2-dev

RUN mkdir -p /usr/simNs3
WORKDIR /usr/simNs3

RUN wget https://www.nsnam.org/release/ns-allinone-3.34.tar.bz2  && \
    tar -jxvf ns-allinone-3.34.tar.bz2

RUN cd ns-allinone-3.34 && ./build.py --enable-examples

RUN apt-get clean && \
    rm -rf /var/lib/apt && \
    rm ns-allinone-3.34.tar.bz2

WORKDIR /usr/simNs3/ns-allinone-3.34/ns-3.34

ADD ./NS3Scenario examples/NS3Scenario/.

COPY runSimulations.py .

RUN  cp examples/NS3Scenario/vanet-test.cc scratch/.

CMD ["python3", "runSimulations.py", "--seed=0"]
