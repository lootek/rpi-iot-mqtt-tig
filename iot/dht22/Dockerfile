FROM python:3.12.0

RUN apt-get update && apt-get install -y \
  build-essential \
  python-dev-is-python3

COPY requirements.txt .
RUN pip install --user -r requirements.txt

COPY adafruit.patch .

RUN git clone https://github.com/adafruit/Adafruit_Python_DHT.git && \
  cd Adafruit_Python_DHT/ && \
  sed -i 's/return None/return 3/g' Adafruit_DHT/platform_detect.py && \
  python setup.py install

#RUN git clone https://github.com/adafruit/Adafruit_Python_DHT.git && \
#  cd Adafruit_Python_DHT/ && \
#  git checkout 8f5e2c4d6ebba8836f6d31ec9a0c171948e3237d && \
#  patch -ubR --verbose Adafruit_DHT/platform_detect.py ../adafruit.patch && \
#  python setup.py install

WORKDIR /code

COPY dht22.py .
CMD ["python", "-u", "/code/dht22.py"]
