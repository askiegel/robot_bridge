#!/usr/bin/env python3

import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Any, Callable, Dict, Optional


class SpeechValidationError(ValueError):
    pass


class SpeechBusyError(RuntimeError):
    pass


class SpeechExecutionError(RuntimeError):
    pass


class SpeechService:
    MAX_TEXT_LENGTH = 300
    ESPEAK_COMMAND = "/usr/bin/espeak"
    APLAY_COMMAND = "/usr/bin/aplay"
    ALSA_DEVICE = "plughw:0,0"
    AMPLITUDE = 70
    SPEED = 145

    def __init__(
        self,
        run_command: Optional[Callable[..., Any]] = None,
    ):
        self._run_command = run_command or subprocess.run
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._last_spoken_text = None
        self._last_spoken_at = None
        self._last_error = None

    def speak(self, text: str, timestamp: str) -> Dict[str, Any]:
        normalized = self._validate_text(text)

        if not self._lock.acquire(blocking=False):
            raise SpeechBusyError(
                "The robot is already speaking."
            )

        wav_path = None

        try:
            with tempfile.NamedTemporaryFile(
                prefix="mini_pupper_speech_",
                suffix=".wav",
                delete=False,
            ) as wav_file:
                wav_path = Path(wav_file.name)

            self._run(
                [
                    self.ESPEAK_COMMAND,
                    "-a",
                    str(self.AMPLITUDE),
                    "-s",
                    str(self.SPEED),
                    "-w",
                    str(wav_path),
                    normalized,
                ],
                timeout=10.0,
                action="Speech synthesis",
            )

            self._run(
                [
                    self.APLAY_COMMAND,
                    "-D",
                    self.ALSA_DEVICE,
                    str(wav_path),
                ],
                timeout=30.0,
                action="Speech playback",
            )

            with self._state_lock:
                self._last_spoken_text = normalized
                self._last_spoken_at = timestamp
                self._last_error = None

            return {
                "ok": True,
                "text": normalized,
                "spoken_at": timestamp,
                "alsa_device": self.ALSA_DEVICE,
            }

        except SpeechExecutionError as exc:
            with self._state_lock:
                self._last_error = str(exc)

            raise

        finally:
            if wav_path is not None:
                try:
                    wav_path.unlink(missing_ok=True)
                except Exception:
                    pass

            self._lock.release()

    def status(self) -> Dict[str, Any]:
        with self._state_lock:
            return {
                "available": True,
                "busy": self._lock.locked(),
                "engine": self.ESPEAK_COMMAND,
                "alsa_device": self.ALSA_DEVICE,
                "max_text_length": self.MAX_TEXT_LENGTH,
                "last_spoken_text": self._last_spoken_text,
                "last_spoken_at": self._last_spoken_at,
                "last_error": self._last_error,
            }

    @classmethod
    def _validate_text(cls, text: str) -> str:
        if not isinstance(text, str):
            raise SpeechValidationError(
                "text must be a string."
            )

        normalized = " ".join(text.strip().split())

        if not normalized:
            raise SpeechValidationError(
                "text cannot be empty."
            )

        if len(normalized) > cls.MAX_TEXT_LENGTH:
            raise SpeechValidationError(
                "text cannot exceed "
                f"{cls.MAX_TEXT_LENGTH} characters."
            )

        return normalized

    def _run(
        self,
        command,
        timeout: float,
        action: str,
    ) -> None:
        try:
            self._run_command(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            raise SpeechExecutionError(
                f"{action} timed out."
            ) from exc
        except subprocess.CalledProcessError as exc:
            stderr = (
                exc.stderr.decode(
                    "utf-8",
                    errors="replace",
                )
                if isinstance(exc.stderr, bytes)
                else str(exc.stderr or "")
            ).strip()

            message = stderr or f"exit status {exc.returncode}"

            raise SpeechExecutionError(
                f"{action} failed: {message}"
            ) from exc
        except OSError as exc:
            raise SpeechExecutionError(
                f"{action} failed: {exc}"
            ) from exc
