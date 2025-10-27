import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time
import pyaudio
import wave
import os
import whisper
import numpy as np
from pathlib import Path
import torch 

# --- CONFIGURACIÓN DE PARÁMETROS ---
DEVICE = "cuda" if torch.cuda.is_available() else "cpu" 

TEMP_AUDIO_DIR = '/tmp/nebu_audio/'
Path(TEMP_AUDIO_DIR).mkdir(parents=True, exist_ok=True) 
WAVE_OUTPUT_FILENAME = os.path.join(TEMP_AUDIO_DIR, "audio_input.wav")

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000 
SILENCE_THRESHOLD = 500 
# 2.0 segundos de silencio para terminar la grabación
SILENCE_CHUNKS = int(RATE / CHUNK * 2.0) 

WHISPER_MODEL = "small" 
WHISPER_LANGUAGE = "es" 


# --- CLASE PRINCIPAL DEL NODO ---
class VoiceTranscriberNode(Node):
    def __init__(self):
        super().__init__('voz_node')

        self.transcription_publisher = self.create_publisher(String, 'transcripcion_voz', 10) 
        
        # SUSCRIPTOR CORREGIDO: Escucha la señal del WAKE_WORD
        self.activation_subscriber = self.create_subscription(
            Bool, 
            'mic_activation_state',  # <-- CORRECCIÓN: Debe escuchar el tópico de activación del WakeNode
            self.activation_callback,
            10
        )

        
        self.mic_active = False
        self.is_recording = False
        self.p = None 
        self.whisper_model = None
        
        self.get_logger().info('Voz Node (Whisper) iniciado. Cargando modelo...')
        self.get_logger().info(f'Modelo Whisper seleccionado: {WHISPER_MODEL} en {DEVICE}')
        threading.Thread(target=self._load_whisper_model).start()


    def _load_whisper_model(self):
        """Carga el modelo Whisper en un thread separado para no bloquear la inicialización de ROS2."""
        try:
            # Carga el modelo pequeño o el configurado
            self.whisper_model = whisper.load_model(WHISPER_MODEL, device=DEVICE)
            self.get_logger().info(f'Modelo Whisper ({WHISPER_MODEL}) cargado en {DEVICE}. listo')
        except Exception as e:
            self.get_logger().error(f' Error al iniciar Whisper: {e}')


    def activation_callback(self, msg): 
        """Callback ejecutado cuando el ESP32 publica un mensaje."""
        # Se activa si el mensaje es True, el micrófono no está activo y el modelo está cargado.
        if msg.data is True and not self.mic_active and self.whisper_model:
            self.mic_active = True
            self.get_logger().info('EL esp32 recibioo la se;al')
            threading.Thread(target=self._record_and_transcribe).start()
        
        # Esta parte no debería ejecutarse si el ESP32 solo envía True, pero es buena práctica.
        elif msg.data is False:
            self.mic_active = False
            self.get_logger().info('Micrófono desactivado.(whisper no activo)')

    
    def _record_and_transcribe(self): 
        """Gestiona la grabación de audio, detección de silencio y transcripción."""
        if self.is_recording:
            return
        self.is_recording = True
        
        # Iniciar PyAudio
        self.p = pyaudio.PyAudio()
        stream = self.p.open(format=FORMAT,
                             channels=CHANNELS,
                             rate=RATE,
                             input=True,
                             frames_per_buffer=CHUNK)

        self.get_logger().info(f"Escuchando y grabando... (Umbral de Silencio: {SILENCE_THRESHOLD})")

        frames = []
        silent_chunks_count = 0
        
        while self.is_recording:
            try:
                # Leer datos de audio
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
                
                # Calcular RMS para detección de actividad de voz
                audio_data = np.frombuffer(data, dtype=np.int16)
                rms = np.sqrt(np.mean(audio_data**2))

                # Detección de Silencio (VAD simple)
                if rms < SILENCE_THRESHOLD:
                    silent_chunks_count += 1
                else:
                    silent_chunks_count = 0 # Reiniciar el contador de silencio

                # LÓGICA DE DETENCIÓN: Si se alcanza el umbral de silencio, detener.
                if silent_chunks_count > SILENCE_CHUNKS:
                    self.is_recording = False

            except Exception as e:
                self.get_logger().error(f"Error durante la grabación: {e}")
                self.is_recording = False

        self.get_logger().info("Grabación finalizada. Procesando con Whisper... ")

        # Cerrar y limpiar la grabación
        stream.stop_stream()
        stream.close()
        self.p.terminate()

        # Guardar audio temporal en .wav
        with wave.open(WAVE_OUTPUT_FILENAME, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))

        transcribed_text = self._transcribe_audio(WAVE_OUTPUT_FILENAME)

        # PUBLICAR RESULTADO
        if transcribed_text:
            self.publish_transcription(transcribed_text)
        else:
             self.publish_transcription("") # Publicar vacío si falla la transcripción
        
        self.mic_active = False
        self.is_recording = False
        
    def _transcribe_audio(self, audio_path):
        """Usa el modelo Whisper para transcribir el archivo de audio."""
        if not self.whisper_model:
            self.get_logger().error('Modelo Whisper no cargado. No se puede transcribir.')
            return None
            
        try:
            result = self.whisper_model.transcribe(
                audio_path, 
                language=WHISPER_LANGUAGE, 
                # Se usa fp16 (media precisión) solo en CUDA para optimizar VRAM y velocidad.
                fp16=True if DEVICE == "cuda" else False 
            )
            text = result["text"].strip()
            self.get_logger().info(f' Transcripcion Whisper: "{text}"')
            return text
        
        except Exception as e:
            self.get_logger().error(f'Error de transcripcion con Whisper: {e}')
            return None


    def publish_transcription(self, text):
        """Publica el texto transcrito en el tópico 'transcripcion_voz'."""
        msg = String()
        msg.data = text
        self.transcription_publisher.publish(msg)
        self.get_logger().info(f'Texto enviado a llm_node: "{text[:50]}..."')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTranscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()