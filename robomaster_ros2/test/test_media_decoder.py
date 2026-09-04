# Copyright 2026 Aarsh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression tests for the memory-safe PyAV video decoder adapter."""

import numpy
import pytest

from robomaster_ros2.modules import media


class FakeFrame:
    """Minimal PyAV video-frame replacement."""

    def __init__(self, image):
        self.image = image
        self.requested_format = None

    def to_ndarray(self, format):
        self.requested_format = format
        return self.image


class FakeCodec:
    """Record incremental parser and decoder calls."""

    def __init__(self, frames):
        self.frames = frames
        self.parsed = []
        self.decoded = []

    def parse(self, data):
        self.parsed.append(data)
        return ["packet"]

    def decode(self, packet):
        self.decoded.append(packet)
        return self.frames


def test_h264_decoder_uses_pyav_incremental_parser_and_bgr_output():
    image = numpy.zeros((2, 3, 3), dtype=numpy.uint8)
    frame = FakeFrame(image)
    codec = FakeCodec([frame])

    result = media.H264Decoder(codec=codec).decode(b"h264 chunk")

    assert codec.parsed == [b"h264 chunk"]
    assert codec.decoded == ["packet"]
    assert frame.requested_format == "bgr24"
    assert len(result) == 1
    assert result[0].shape == (2, 3, 3)
    assert result[0].flags.c_contiguous


def test_h264_decoder_rejects_invalid_frame_layout():
    codec = FakeCodec([FakeFrame(numpy.zeros((2, 3), dtype=numpy.uint8))])

    with pytest.raises(ValueError, match="invalid BGR"):
        media.H264Decoder(codec=codec).decode(b"h264 chunk")


def test_h264_decoder_explains_missing_pyav(monkeypatch):
    monkeypatch.setattr(media, "av", None)

    with pytest.raises(RuntimeError, match="python3-av"):
        media.H264Decoder()
