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

"""Remote Piper "dumb executor" bridge.

This package holds the socket bridge between the RLinf server (which runs the
policy) and a remote robot machine (which only executes actions and returns
observations). Gym registration and the heavier env wrappers are added by later
tasks; keep imports light here so :mod:`protocol` can be copied to the robot
machine standalone.
"""
