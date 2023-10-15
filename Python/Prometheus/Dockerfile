FROM ubuntu
FROM python:3.8

RUN pip install pyserial &&\
    pip install pyyaml &&\
    pip install prometheus_client

WORKDIR /app

COPY . /app

CMD ["python", "read_uart.py"]
