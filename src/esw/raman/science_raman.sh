#!/bin/bash

/usr/bin/socat pty,link=/dev/ttyACM0,waitslave tcp:10.1.0.2:8888