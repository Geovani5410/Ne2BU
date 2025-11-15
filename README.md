
# Guia de instalacion Ne2BU

Para base de este proyecto esta basada en ROS2 Humble por lo cual es necesario antes instalar algunas dependencias y soporte. 






## Instalacion de ROS2 Humble - Ubunto

ROS2 Es un software que permite la comunicacion entre nodos, topic entre software y hadware
Para su instalacion puedes vistiar la documentacion oficial: [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#id8)


o desde nuestro repositorio con la guia de instalacion para ROS2 





## Instalacion de LLM - llama3
Llama3 es la cereza del pastel, utilizamos este modelo de lenguaje largo para obtener respuestas. 
Para la instalacion es Ubunto: 

```bash
  curl -fsSL https://ollama.com/install.sh | sh
```
una vez terminado instalar el modelo de lenguaje especifico: 

```bash
  ollama run llama3
```

##Dependencias del sistema (Ubunto)
```bash
sudo apt update
sudo apt install python3-pyaudio libasound-dev mpg123
```




## Dependencias de python 
Esta libreria nos permitiran trabajar con los nodos
```bash
pip install rclpy
pip install sounddevice
pip install vosk
pip install pyaudio
pip install torch
pip install numpy
pip install git+https://github.com/openai/whisper.git
pip install numba==0.57.1
pip install coverage==6.5.0
pip install requests
pip install pygame
```







