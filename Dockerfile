FROM ubuntu:18.04

RUN apt-get update -y
RUN apt-get install -y curl

WORKDIR /
RUN curl -LO http://download.atollic.com/TrueSTUDIO/installers/Atollic_TrueSTUDIO_for_STM32_linux_x86_64_v9.3.0_20190212-0734.tar.gz
RUN tar -xvf Atollic_TrueSTUDIO_for_STM32_linux_x86_64_v9.3.0_20190212-0734.tar.gz

WORKDIR /Atollic_TrueSTUDIO_for_STM32_9.3.0_installer
RUN yes 1 | ./install.sh

COPY . /aisaac-stm32-main
WORKDIR /opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0/ide/
CMD ./headless.sh -data /workspace -import /aisaac-stm32-main -build main_stm/Debug
