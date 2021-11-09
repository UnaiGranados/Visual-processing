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

echo_and_run cd "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/geometry_msgs_helper_functions"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python2.7/dist-packages:/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing" \
    "/usr/bin/python2" \
    "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/geometry_msgs_helper_functions/setup.py" \
     \
    build --build-base "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/geometry_msgs_helper_functions" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
