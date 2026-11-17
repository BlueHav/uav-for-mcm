#!/usr/bin/env bash
set -e
# Applied only to RViz, leaving Unity and planner environments unchanged.
export QT_X11_NO_MITSHM=1
case "${YOPO_RVIZ_RENDERER:-gpu}" in
  gpu)
    unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    ;;
  software)
    export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa
    ;;
  *) echo 'YOPO_RVIZ_RENDERER must be gpu or software' >&2; exit 2 ;;
esac
echo "RViz renderer mode: ${YOPO_RVIZ_RENDERER:-gpu}; DISPLAY=${DISPLAY:-unset}" >&2
exec "$@"
