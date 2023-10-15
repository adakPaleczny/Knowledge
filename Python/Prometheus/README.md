# Python data exporter for Docker Prometheus

This python script uses Promethues to export data into Prometheus in Docker. 

Uses **promethues_client** library and Guage and Enum messages in Promethues.

## Configuration
In **prometheus_read_uart.yaml** you can chose baud rate and device from which you can read message. Also it is able to disable promethues exporter by changing:
```yaml
    use_prometheus: False
```
In yaml file.