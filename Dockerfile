# To build use:
# docker build -t orian/cppenv:v1 .
FROM microsoft/windowsservercore
MAINTAINER Zhang Yiming <public at buaazimmy@live.com>

LABEL description="A win32 uart exe."

RUN make

CMD ["/bin/bash"]
