#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/aims/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/aims/catkin_ws/install/lib/python2.7/dist-packages:/home/aims/catkin_ws/build/dji_sdk/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/aims/catkin_ws/build/dji_sdk" \
    "/usr/bin/python2" \
    "/home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/setup.py" \
     \
    build --build-base "/home/aims/catkin_ws/build/dji_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/aims/catkin_ws/install" --install-scripts="/home/aims/catkin_ws/install/bin"
