import httpx
import sounddevice as sd
import numpy as np
import io
import wave
import argparse

ENDPOINTS = {
    "customvoice": "http://localhost:8091",
    "voicedesign": "http://localhost:8092",
    "base": "http://localhost:8093",
}

def synthesize_and_play(
    text: str,
    voice: str = "Vivian",
    model_type: str = "customvoice",
    language: str = "English",
    instruct: str = "",
    base_url: str | None = None,
):
    url = (base_url or ENDPOINTS[model_type]) + "/v1/audio/speech"

    model_map = {
        "customvoice": "Qwen/Qwen3-TTS-12Hz-0.6B-CustomVoice",
        "voicedesign": "Qwen/Qwen3-TTS-12Hz-0.6B-VoiceDesign",
        "base": "Qwen/Qwen3-TTS-12Hz-0.6B-Base",
    }

    payload = {
        "model": model_map[model_type],
        "input": text,
        "voice": voice,
        "language": language,
        "response_format": "wav",
    }
    if instruct:
        payload["instruct"] = instruct

    print(f"[→] Отправка запроса к {url} ...")

    with httpx.Client(timeout=120) as client:
        response = client.post(url, json=payload)
        response.raise_for_status()
        audio_bytes = response.content

    # Читаем WAV из памяти — без записи на диск
    with wave.open(io.BytesIO(audio_bytes)) as wf:
        sample_rate = wf.getframerate()
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()   # байт на семпл
        frames = wf.readframes(wf.getnframes())

    # Конвертируем байты → numpy array
    dtype_map = {1: np.int8, 2: np.int16, 4: np.int32}
    dtype = dtype_map.get(sample_width, np.int16)
    audio_np = np.frombuffer(frames, dtype=dtype)

    if n_channels > 1:
        audio_np = audio_np.reshape(-1, n_channels)

    # Нормализуем до float32 [-1.0, 1.0] для sounddevice
    audio_float = audio_np.astype(np.float32) / np.iinfo(dtype).max

    duration = len(audio_float) / sample_rate
    print(f"[♪] Воспроизведение: {duration:.1f} сек | {sample_rate} Hz | {n_channels}ch")

    sd.play(audio_float, samplerate=sample_rate)
    sd.wait()   # блокируемся до конца воспроизведения
    print("[✓] Готово")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Qwen3-TTS клиент")
    parser.add_argument("--text", default="Hello! This is Qwen3-TTS speaking.")
    parser.add_argument("--voice", default="Vivian",
                        help="Имя голоса (CustomVoice) или описание (VoiceDesign)")
    parser.add_argument("--type", default="customvoice",
                        choices=["customvoice", "voicedesign", "base"])
    parser.add_argument("--language", default="English")
    parser.add_argument("--instruct", default="",
                        help="Дополнительная инструкция: 'speak slowly', 'angry tone' и т.п.")
    parser.add_argument("--url", default=None,
                        help="Переопределить базовый URL, напр. http://myserver:8091")
    args = parser.parse_args()

    synthesize_and_play(
        text=args.text,
        voice=args.voice,
        model_type=args.type,
        language=args.language,
        instruct=args.instruct,
        base_url=args.url,
    )
    