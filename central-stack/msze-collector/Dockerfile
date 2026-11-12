# Holy Mass schedule collector: nightly LLM extraction of church pages into a
# JSON cache, plain JSON API on :8091 for the Grafana Infinity datasource.
# Built on the Pi (aarch64); all dependencies are pure Python or ship wheels.
FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY *.py churches.yaml ./

EXPOSE 8091
CMD ["python", "app.py"]
