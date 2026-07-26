#!/usr/bin/env python3

import app as bridge_app
from speech_service import (
    SpeechBusyError,
    SpeechExecutionError,
    SpeechValidationError,
)


class FakeSpeechService:
    def __init__(self):
        self.calls = []
        self.error = None

    def speak(self, text, timestamp):
        self.calls.append(
            {
                "text": text,
                "timestamp": timestamp,
            }
        )

        if self.error is not None:
            raise self.error

        return {
            "ok": True,
            "text": text,
            "spoken_at": timestamp,
            "alsa_device": "plughw:0,0",
        }

    def status(self):
        return {
            "available": True,
            "busy": False,
            "alsa_device": "plughw:0,0",
        }


def main():
    original = bridge_app.speech_service
    fake = FakeSpeechService()
    bridge_app.speech_service = fake

    try:
        client = bridge_app.app.test_client()

        response = client.post(
            "/speak",
            json={"text": "Hello Tony."},
        )
        payload = response.get_json()

        assert response.status_code == 200
        assert payload["ok"] is True
        assert payload["action"] == "speak"
        assert fake.calls[-1]["text"] == "Hello Tony."
        print("PASS: valid speech request reaches speech service")

        response = client.get("/status")
        payload = response.get_json()
        assert payload["speech"]["available"] is True
        assert payload["speech"]["alsa_device"] == "plughw:0,0"
        print("PASS: status exposes speech capability")

        response = client.post(
            "/speak",
            data="not json",
            content_type="text/plain",
        )
        assert response.status_code == 400
        print("PASS: non-JSON speech request is rejected")

        for error, status_code, message in (
            (
                SpeechValidationError("bad text"),
                400,
                "validation error maps to HTTP 400",
            ),
            (
                SpeechBusyError("already speaking"),
                409,
                "busy error maps to HTTP 409",
            ),
            (
                SpeechExecutionError("audio failed"),
                503,
                "audio failure maps to HTTP 503",
            ),
        ):
            fake.error = error
            response = client.post(
                "/speak",
                json={"text": "Hello"},
            )
            assert response.status_code == status_code
            print(f"PASS: {message}")

        print()
        print("Speech endpoint test passed.")

    finally:
        bridge_app.speech_service = original


if __name__ == "__main__":
    main()
