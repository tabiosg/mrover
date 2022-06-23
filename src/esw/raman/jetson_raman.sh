#!/bin/bash

/usr/bin/socat tcp-listen:8888,reuseaddr,fork file:/dev/raman,nonblock,waitlock=/var/run/tty0.lock,b115200,raw,echo=0