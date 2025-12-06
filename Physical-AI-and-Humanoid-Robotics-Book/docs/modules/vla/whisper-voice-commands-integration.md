---
id: whisper-voice-commands-integration
title: Whisper Voice Commands Integration"
slug: /modules/vla/whisper-voice-commands-integration
---

# Whisper Voice Commands Integration

Natural Language Interaction (NLI) is a pivotal step towards truly intuitive human-robot collaboration. While traditional robotic interfaces often rely on joysticks, graphical user interfaces, or predefined gestures, the ability to simply tell a robot what to do, using everyday language, unlocks a vast potential for accessibility, efficiency, and seamless integration into human environments. For humanoid robots, which are designed to operate in human spaces, voice commands offer a particularly natural and powerful mode of interaction.

This chapter delves into integrating **OpenAI's Whisper model** for robust voice command recognition, transforming spoken instructions into actionable text commands for our humanoid robot within the ROS 2 ecosystem.

## 1. The Challenge of Speech Recognition in Robotics

Accurate Automatic Speech Recognition (ASR) is a complex task, especially in real-world robotic environments which are often noisy. Key challenges include:
*   **Background Noise:** Ambient sounds, motor hums, human chatter, and environmental disturbances can degrade recognition accuracy.
*   **Speaker Variability:** Different accents, speaking styles, and voice pitches.
*   **Terminology:** Robotic applications often use specialized jargon.
*   **Latency:** For real-time control, speech needs to be transcribed quickly.
*   **Privacy:** On-device processing vs. cloud-based services.

Historically, ASR systems struggled with these challenges. However, recent advancements, particularly with large, pre-trained models, have dramatically improved performance.

## 2. OpenAI Whisper: A Robust ASR Model

**OpenAI Whisper** is a general-purpose, multilingual, and multitask ASR model. It was trained on a massive dataset of diverse audio and corresponding English transcripts, making it highly robust to various accents, background noise, and technical language.

### A. Key Features of Whisper

*   **Robustness:** Excels in noisy environments and with diverse speech patterns.
*   **Multilingual:** Can transcribe and translate many languages.
*   **Multitask:** Can perform language identification, voice activity detection, and speech translation in addition to ASR.
*   **Open Source:** OpenAI has released the model and inference code, allowing for local deployment.
*   **Model Sizes:** Available in various sizes (tiny, base, small, medium, large), offering a trade-off between speed/resource usage and accuracy.

**Figure 2.1: Whisper Model Overview**
```
+-----------------------------------------------------------+
|                      Audio Input                          |
|  (Raw Audio Stream from Microphone)                       |
+-----------------------------------------------------------+
          |
          v
+-----------------------------------------------------------+
|             OpenAI Whisper Model (Local or API)           |
|  +-----------------------------------------------------+  |
|  |             Feature Extraction (Encoder)            |  |
|  |             - Mel Spectrogram                       |  |
|  +-----------------------------------------------------+  |
|                            |                              |
|                            v                              |
|  +-----------------------------------------------------+  |
|  |             Sequence-to-Sequence Model (Decoder)    |  |
|  |             - Transformer-based Architecture        |  |
|  |             - Trained on huge audio-text dataset    |  |
|  +-----------------------------------------------------+  |
|                            |                              |
|                            v                              |
+-----------------------------------------------------------+
|                     Text Output                           |
|  (Transcribed Speech: "Robot, go to the kitchen.")        |
+-----------------------------------------------------------+
```

## 3. Integrating Whisper with ROS 2

For real-time voice command processing, we need a ROS 2 node that captures audio, feeds it to Whisper, and then publishes the transcribed text to a ROS 2 topic.

### A. Audio Input Node

We need a node that captures audio from a microphone. The `audio_common` package in ROS 2 can provide such functionality.

```bash
# Example for installing audio_common
sudo apt install ros-humble-audio-common
```

The `audio_capture` node from `audio_common` can publish raw audio data to a ROS 2 topic (e.g., `/audio/audio`).

### B. Whisper ROS 2 Bridge Node

We will create a custom ROS 2 Python node that:
1.  Subscribes to the raw audio topic (`/audio/audio`).
2.  Buffers the audio for a short duration (e.g., 2-5 seconds).
3.  Feeds the buffered audio to a locally running Whisper model.
4.  Publishes the transcribed text to a new ROS 2 topic (e.g., `/humanoid/voice_commands`).

**Conceptual Code: `humanoid_vla/humanoid_vla/whisper_asr_node.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
import io
import soundfile as sf
import threading
import torch

# Make sure you have the 'whisper' Python package installed:
# pip install -U openai-whisper
# pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 # For GPU
# pip install soundfile

class WhisperASRNode(Node):
    def __init__(self):
        super().__init__('whisper_asr_node')
        self.declare_parameter('whisper_model_size', 'base')
        self.declare_parameter('audio_topic', '/audio/audio')
        self.declare_parameter('command_topic', '/humanoid/voice_commands')
        self.declare_parameter('sample_rate', 16000) # Whisper expects 16kHz
        self.declare_parameter('buffer_duration_sec', 5.0) # Buffer 5 seconds of audio
        self.declare_parameter('min_audio_length_sec', 1.0) # Minimum audio length to process

        model_size = self.get_parameter('whisper_model_size').get_parameter_value().string_value
        audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.buffer_duration_sec = self.get_parameter('buffer_duration_sec').get_parameter_value().double_value
        self.min_audio_length_sec = self.get_parameter('min_audio_length_sec').get_parameter_value().double_value

        self.get_logger().info(f"Loading Whisper model '{model_size}'...")
        # Check if CUDA is available for GPU inference
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size, device=self.device)
        self.get_logger().info(f"Whisper model loaded on {self.device}.")

        self.audio_buffer = []
        self.audio_buffer_lock = threading.Lock()
        self.last_audio_time = self.get_clock().now().nanoseconds / 1e9

        self.subscription = self.create_subscription(
            AudioData,
            audio_topic,
            self.audio_callback,
            10 # QoS profile depth
        )
        self.command_publisher = self.create_publisher(String, command_topic, 10)

        # Timer to periodically process the audio buffer
        self.process_timer = self.create_timer(1.0, self.process_audio_buffer) # Process buffer every 1 sec

    def audio_callback(self, msg: AudioData):
        """Callback for incoming raw audio data."""
        with self.audio_buffer_lock:
            # Assuming AudioData.data is a byte array representing raw PCM audio
            # Convert bytes to numpy array (int16 assumed, adjust if different)
            audio_np = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32) / 32768.0 # Normalize to float32
            self.audio_buffer.extend(audio_np.tolist())
            self.last_audio_time = self.get_clock().now().nanoseconds / 1e9

            # Trim buffer to max duration
            max_samples = int(self.sample_rate * self.buffer_duration_sec)
            if len(self.audio_buffer) > max_samples:
                self.audio_buffer = self.audio_buffer[-max_samples:]

    def process_audio_buffer(self):
        """Periodically processes the buffered audio for transcription."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.audio_buffer_lock:
            # Only process if there's enough new audio or a long silence
            if len(self.audio_buffer) == 0:
                return

            audio_duration = len(self.audio_buffer) / self.sample_rate
            if audio_duration < self.min_audio_length_sec and (current_time - self.last_audio_time < 0.5): # Wait for more audio or slight pause
                return

            self.get_logger().info(f"Processing {audio_duration:.2f} seconds of audio...")

            # Convert buffer to numpy array for Whisper
            audio_input = np.array(self.audio_buffer, dtype=np.float32)

            # Whisper expects 16kHz mono audio. If source is different, resample here.
            # Assuming 'audio_common_msgs/AudioData' is 16kHz mono PCM.

            try:
                result = self.model.transcribe(audio_input, fp16=(self.device == "cuda"))
                transcribed_text = result["text"].strip()
                if transcribed_text:
                    self.get_logger().info(f"Transcribed: '{transcribed_text}'")
                    msg = String()
                    msg.data = transcribed_text
                    self.command_publisher.publish(msg)
                else:
                    self.get_logger().debug("Whisper transcribed empty text.")
            except Exception as e:
                self.get_logger().error(f"Error during Whisper transcription: {e}")
            finally:
                self.audio_buffer = [] # Clear buffer after processing

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperASRNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*   **Dependencies:** `openai-whisper`, `torch`, `torchaudio`, `soundfile`.
*   **`setup.py`:** Add `whisper_asr_node` as a console script entry point in your `humanoid_vla` package.

### C. Launching the Whisper ASR Pipeline

1.  **Start Audio Capture:**
    ```bash
    ros2 run audio_capture audio_capture_node --ros-args -p audio_device_name:="hw:0,0" -p format:=S16LE -p channels:=1 -p sample_rate:=16000 -r audio:=/audio/audio
    ```
    (Adjust `audio_device_name` to your microphone. `hw:0,0` is often the default.)
2.  **Start Whisper ASR Node:**
    ```bash
    ros2 run humanoid_vla whisper_asr_node
    ```
3.  **Monitor Commands:**
    ```bash
    ros2 topic echo /humanoid/voice_commands
    ```

Now, when you speak into your microphone, Whisper will transcribe your speech, and the transcribed text will appear on the `/humanoid/voice_commands` topic.

## 4. On-Device vs. Cloud-Based Whisper

The `whisper` Python package can run inference locally on your CPU or GPU (if CUDA is available). For robotic applications, especially on edge devices like NVIDIA Jetson, local inference is often preferred for:
*   **Low Latency:** Faster response times without network roundtrips.
*   **Privacy:** Audio data doesn't leave the device.
*   **Offline Capability:** Works without an internet connection.

However, running larger Whisper models (e.g., `large`) on resource-constrained Jetson devices might be too slow. A balance between model size, accuracy, and inference speed must be found. Optimized versions of Whisper for NVIDIA hardware (e.g., via TensorRT) can significantly improve performance.

### A. Comparison: Local vs. Cloud Whisper

| Feature       | Local (On-Device) Inference                                   | Cloud-Based API (e.g., OpenAI API)                                |
| :------------ | :------------------------------------------------------------ | :------------------------------------------------------------------ |
| **Latency**   | Lower (no network latency)                                    | Higher (depends on network speed)                                   |
| **Privacy**   | High (audio stays on device)                                  | Lower (audio sent to cloud provider)                                |
| **Cost**      | One-time hardware/energy cost                                 | Per-use API cost                                                    |
| **Offline**   | Yes                                                           | No (requires internet)                                              |
| **Hardware**  | Requires capable local CPU/GPU (Jetson or workstation)        | Offloaded to cloud, minimal local compute required                  |
| **Scalability** | Limited by local hardware resources                           | Highly scalable (cloud handles demand)                              |
| **Deployment** | More complex (model packaging, dependencies)                  | Simpler (API key, HTTP requests)                                    |
| **Model Size** | Constrained by local memory/compute                           | Can use largest, most accurate models                               |

## 5. Case Study: Voice Control for Humanoid Task Initiation

**Scenario:** A human wants to initiate a task on the humanoid robot using a simple voice command, like "Robot, pick up the red cube."

**Whisper's Role:**
1.  **Audio Capture:** A ReSpeaker USB mic array (as recommended in hardware requirements) captures the human's voice.
2.  **ROS 2 Audio Node:** The `audio_capture` node publishes the raw audio stream to `/audio/audio`.
3.  **Whisper ASR Node:** Our `whisper_asr_node` subscribes to `/audio/audio`, transcribes "Robot, pick up the red cube," and publishes this text to `/humanoid/voice_commands`.
4.  **LLM Planning Node (Next Chapter):** Another ROS 2 node (an LLM planner) subscribes to `/humanoid/voice_commands`, interprets the intent, and translates it into a sequence of robotic actions.

This setup provides the critical first step in a Vision-Language-Action (VLA) pipeline, enabling natural and intuitive control of our humanoid robot.

## Conclusion

Integrating OpenAI's Whisper model into a ROS 2 system provides a robust and powerful solution for voice command recognition in humanoid robots. By capturing audio, transcribing it efficiently (ideally on-device for latency and privacy), and publishing the resulting text to ROS 2 topics, we enable a natural language interface that is crucial for advanced human-robot interaction and high-level task planning. This forms a foundational component of the Vision-Language-Action capabilities we will explore.