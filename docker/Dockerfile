FROM --platform=linux/arm64 ubuntu:24.04

# Ubuntu 24.04 ships every version we need with no source builds:
#   GCC 13.3, CMake 3.28.3, OpenCV 4.6, Eigen 3.4, yaml-cpp 0.8, Boost 1.83,
#   GLEW 2.2, Mesa >= 24 (llvmpipe reports GL 4.5 compatibility).
# Do NOT add the Kitware CMake repo: CMake >= 4 removed compatibility with
# cmake_minimum_required(<3.5), which the vendored third-party CMakeLists still use.
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config git ca-certificates patch rsync \
      libeigen3-dev libopencv-dev libyaml-cpp-dev libboost-dev \
      libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
      libgl1-mesa-dri mesa-utils \
      libqt5gui5 libqt5widgets5 \
      libx11-6 libxi6 libxrandr2 libxxf86vm1 libxcursor1 libxinerama1 x11-utils \
      xvfb x11vnc novnc websockify python3-numpy \
      x11-apps imagemagick \
 && (apt-get install -y --no-install-recommends libglut-dev \
     || apt-get install -y --no-install-recommends freeglut3-dev) \
 && pkg-config --exists glut \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY third_party/rovio                    third_party/rovio
COPY third_party/kindr/include/kindr      third_party/kindr/include/kindr
COPY third_party/lightweight_filtering/include/lightweight_filtering \
                                          third_party/lightweight_filtering/include/lightweight_filtering
COPY contrib                              contrib
COPY fixes                                fixes
COPY scripts                              scripts
COPY rosfree                              rosfree
COPY cfg                                  cfg

# Layer the sources, then build. Two binaries: the 2-camera one and a single-camera
# variant used for the visual-only / thermal-only bring-up stages.
RUN bash scripts/stage_sources.sh /src/staged \
 && cmake -S rosfree -B build2 -DCMAKE_BUILD_TYPE=Release \
      -DROVTIO_STAGED=/src/staged -DMAKE_SCENE=ON -DROVIO_NCAM=2 \
 && cmake --build build2 -j"$(nproc)" \
 && cmake --install build2 --prefix /opt/rovtio \
 && cmake -S rosfree -B build1 -DCMAKE_BUILD_TYPE=Release \
      -DROVTIO_STAGED=/src/staged -DMAKE_SCENE=ON -DROVIO_NCAM=1 \
 && cmake --build build1 -j"$(nproc)" \
 && cp build1/rovtio_player /opt/rovtio/bin/rovtio_player_mono \
 && cp -r /src/cfg /opt/rovtio/cfg

# llvmpipe software rendering: there is no GPU passthrough to Linux containers on macOS.
# GALLIUM_DRIVER must be llvmpipe (softpipe is ~50x slower); LIBGL_ALWAYS_INDIRECT must
# stay unset or the ESSL 3.00 shaders fail to compile against XQuartz's GL 1.4 GLX.
ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    LP_NUM_THREADS=8 \
    QT_X11_NO_MITSHM=1 \
    MESA_GL_VERSION_OVERRIDE=4.5COMPAT \
    MESA_GLSL_VERSION_OVERRIDE=450

# The Ubuntu novnc package stamps its files 2021-10-22 and websockify sends no
# Cache-Control, so a browser's heuristic freshness (10% of the file's age) is ~176
# days: it will run a previously cached app/ui.js WITHOUT revalidating. If any other
# project ever served a different noVNC on the same host port, that stale ui.js is
# what executes -- against our newer vnc.html -- and it dies on a missing element.
# Restamping to build time makes heuristic freshness ~0, so the browser revalidates.
RUN find /usr/share/novnc -type f -exec touch {} +

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/rovtio/bin/rovtio_player", "--help"]
