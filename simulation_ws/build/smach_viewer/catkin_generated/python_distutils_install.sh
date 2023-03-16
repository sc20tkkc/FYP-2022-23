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

echo_and_run cd "/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/src/smach_viewer"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/install/lib/python3/dist-packages:/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/build" \
    "/usr/bin/python3" \
    "/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/src/smach_viewer/setup.py" \
    egg_info --egg-base /home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/build/smach_viewer \
    build --build-base "/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/build/smach_viewer" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/install" --install-scripts="/home/csunix/sc20tkkc/FYP-2022-23/simulation_ws/install/bin"
