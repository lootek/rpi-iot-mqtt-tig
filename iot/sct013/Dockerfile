FROM python:3.12.0

RUN apt-get update && apt-get install -y \
    build-essential \
    python-dev-is-python3 \
    python3-pip \
    python3-pil \
    python3-numpy \
    ttf-wqy-zenhei \
    python3-spidev \
    python3-RPi.GPIO \
    doxygen

#RUN wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz && \
#    tar zxvf bcm2835-1.71.tar.gz && \
#    cd bcm2835-1.71/ && \
#    ./configure && make && make check && make install

#RUN git clone https://github.com/WiringPi/WiringPi && \
#    cd WiringPi/ && \
#    . ./build && \
#    gpio -v

#RUN wget https://github.com/joan2937/lg/archive/master.zip && \
#    unzip master.zip && \
#    cd lg-master/ && \
#    make install

COPY requirements.txt .
RUN pip install --user -r requirements.txt

RUN git clone https://github.com/waveshare/High-Pricision_AD_HAT && \
    cp High-Pricision_AD_HAT/python/*.py .

COPY High-Precision-AD-HAT.patch .
RUN patch < High-Precision-AD-HAT.patch

COPY sct013.py .

CMD ["python", "-u", "sct013.py"]
