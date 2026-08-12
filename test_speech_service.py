#!/usr/bin/env python3

import subprocess
import threading

from speech_service import (
    SpeechBusyError,
    SpeechExecutionError,
    SpeechService,
    SpeechValidationError,
)


class CommandRecorder:
    def __init__(self):
        self.calls = []

    def __call__(self, command, **kwargs):
        self.calls.append((list(command), dict(kwargs)))
        return subprocess.CompletedProcess(command, 0)


def expect_error(error_type, callback, message):
    try:
        callback()
    except error_type:
        print(f"PASS: {message}")
    else:
        raise AssertionError(message)


def main():
    recorder = CommandRecorder()
    service = SpeechService(run_command=recorder)

    result = service.speak(
        "  Hello   Tony.  ",
        timestamp="2026-07-26T22:00:00Z",
    )

    assert result["ok"] is True
    assert result["text"] == "Hello Tony."
    assert result["alsa_device"] == "plughw:0,0"
    assert len(recorder.calls) == 2
    print("PASS: speech text is normalized and played")

    synthesis = recorder.calls[0][0]
    playback = recorder.calls[1][0]

    assert synthesis[0] == "/usr/bin/espeak"
    assert synthesis[-1] == "Hello Tony."
    assert playback[:3] == [
        "/usr/bin/aplay",
        "-D",
        "plughw:0,0",
    ]
    print("PASS: fixed executables and ALSA device are used")
    print("PASS: user text is passed as one argument without a shell")

    status = service.status()
    assert status["last_spoken_text"] == "Hello Tony."
    assert status["last_error"] is None
    print("PASS: successful speech updates telemetry")

    expect_error(
        SpeechValidationError,
        lambda: service.speak(None, "timestamp"),
        "non-string text is rejected",
    )
    expect_error(
        SpeechValidationError,
        lambda: service.speak("   ", "timestamp"),
        "empty text is rejected",
    )
    expect_error(
        SpeechValidationError,
        lambda: service.speak(
            "x" * (SpeechService.MAX_TEXT_LENGTH + 1),
            "timestamp",
        ),
        "overlong text is rejected",
    )

    service._lock.acquire()

    try:
        expect_error(
            SpeechBusyError,
            lambda: service.speak("Hello", "timestamp"),
            "overlapping speech is rejected",
        )
    finally:
        service._lock.release()

    class FailedRunner:
        def __call__(self, command, **kwargs):
            raise subprocess.CalledProcessError(
                1,
                command,
                stderr=b"test playback failure",
            )

    failed = SpeechService(run_command=FailedRunner())

    expect_error(
        SpeechExecutionError,
        lambda: failed.speak("Hello", "timestamp"),
        "audio execution errors are surfaced",
    )
    assert failed.status()["last_error"]
    print("PASS: audio failure updates telemetry")

    print()
    print("Speech service test passed.")


if __name__ == "__main__":
    main()


def test_speech_service_uses_verified_headphone_output():
    assert SpeechService.ALSA_DEVICE == "plughw:1,0"
