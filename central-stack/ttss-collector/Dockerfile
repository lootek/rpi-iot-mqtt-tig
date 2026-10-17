# ZTP GTFS-RT departures collector: caches the protobuf feeds in memory and
# serves plain JSON for the Grafana Infinity datasource. Built on the Pi
# (aarch64); everything here is pure Python, so slim stays small.
FROM python:3.12-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY app.py .

EXPOSE 8090
CMD ["python", "app.py"]
