#--------------
FROM cern/cc7-base as tpc-fec-utils-cc7-base
COPY contrib/alisw-el7.repo /etc/yum.repos.d/alisw-el7.repo
RUN yum install -y \
  yum-plugin-ovl \
  git \
  gcc \
  make \
  cmake3 \
  openssh-clients \
  centos-release-scl-rh \
  devtoolset-4-gcc-c++ \
  devtoolset-6 \
  devtoolset-7 \
  boost-devel \
  cppcheck \
  mariadb-devel \
  environment-modules \
  rpm-build \
  CERN-CA-certs \
  pciutils-devel


#--------------
FROM tpc-fec-utils-cc7-base as tpc-fec-utils-cc7-readoutcard-0.8.8
RUN yum install -y \
  alisw-ReadoutCard+v0.8.8-3
WORKDIR /ci-build
