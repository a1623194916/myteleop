# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Rising-edge subtask tracker for the PiperX teleop recorder.

Pure logic (no hardware imports). The operator presses the left-X controller
button to mark the current frame as a subtask boundary; each *rising edge*
advances an index through a predefined, ordered subtask list. The index is
clamped to ``len(subtasks) - 1`` (it never wraps or overflows).

The tracker is shared between the button-poll thread (which calls ``update``)
and the EpisodeRecorder's ``extra_fn`` (which reads ``current_index`` every
sample), so reads and writes are guarded by a lock.
"""

import threading
from typing import Sequence


class SubtaskTracker:
    """Advance a bounded subtask index on each left-X rising edge."""

    def __init__(self, subtasks: Sequence[str]):
        subtasks = list(subtasks)
        if not subtasks:
            raise ValueError("subtasks must be a non-empty ordered list")
        self._subtasks: list[str] = [str(s) for s in subtasks]
        self._idx = 0
        self._prev_pressed = False
        self._lock = threading.Lock()

    def update(self, pressed: bool) -> bool:
        """Feed the current left-X button state.

        Returns ``True`` iff this call advanced the subtask index (i.e. a
        rising edge occurred and the index was not already at the end).
        """
        pressed = bool(pressed)
        with self._lock:
            advanced = False
            if pressed and not self._prev_pressed:
                if self._idx < len(self._subtasks) - 1:
                    self._idx += 1
                    advanced = True
            self._prev_pressed = pressed
            return advanced

    def reset(self) -> None:
        """Reset the index to 0 for a new episode.

        The button edge state is preserved: if X is still physically held
        across the reset, no phantom edge fires on the next ``update``.
        """
        with self._lock:
            self._idx = 0

    @property
    def current_index(self) -> int:
        with self._lock:
            return self._idx

    @property
    def current_name(self) -> str:
        with self._lock:
            return self._subtasks[self._idx]

    @property
    def subtasks(self) -> list[str]:
        return list(self._subtasks)

    def __len__(self) -> int:
        return len(self._subtasks)
