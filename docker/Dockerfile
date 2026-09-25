# Два образа из одного файла (D-004, D-010):
#   jury — то, что мы считаем окружением жюри: чистый ros:humble-ros-base, без доустановок.
#          Рантайм нод обязан работать здесь: только rclpy + numpy (python3-numpy 1.21 есть в базе).
#   dev  — рабочее место участника: + scipy, pytest, rosbags для tools/ и тестов ядра на numpy 1.21.
#          Графики (matplotlib, pandas) — в .venv на хосте: Qt/Tk в образе стоят ~1 ГБ диска.
# Сборка: docker build -f docker/Dockerfile --target <jury|dev> -t tram-odom:<jury|dev> .
FROM ros:humble-ros-base AS jury
SHELL ["/bin/bash", "-c"]
# procps — ps/top для замера CPU/RAM на стенде; time — пиковый RSS.
RUN apt-get update && apt-get install -y --no-install-recommends procps time \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws

FROM jury AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-scipy python3-yaml python3-pytest git less \
    && rm -rf /var/lib/apt/lists/*
COPY requirements-dev.txt /tmp/requirements-dev.txt
# numpy закреплён на версии из apt (как у жюри): pip не должен её обновить.
RUN pip3 install --no-cache-dir "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')" \
      "rosbags==$(grep -oP 'rosbags==\K.*' /tmp/requirements-dev.txt)"
RUN echo 'source /opt/ros/humble/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /etc/bash.bashrc
