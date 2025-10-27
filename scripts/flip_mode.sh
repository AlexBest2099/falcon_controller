#!/bin/bash
source ~/ros/devel/setup.bash

rosservice call ./ModeChange "orientation:
- {data: 180.0}
- {data: -180.0}
- {data: 90.0}"

