# Copyright 2026 Pixel Robotics GmbH
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

"""Tests for to_parameters_list with parameter files (parsed once while unchanged)."""

import importlib
import os
import pathlib

from launch import LaunchContext
import pytest
import yaml

# the module: launch_ros.utilities re-exports the function under the same name
module = importlib.import_module('launch_ros.utilities.to_parameters_list')

PARAMS = """
/**:
  ros__parameters:
    shared: 1
    nested:
      value: [1.0, 2.0]
my_node:
  ros__parameters:
    own: 'x'
other_node:
  ros__parameters:
    own: 'y'
"""


@pytest.fixture(autouse=True)
def empty_cache(monkeypatch):
    monkeypatch.setattr(module, '_parameter_file_cache', {})


def _as_dict(parameters):
    return {p.name: p.value for p in parameters}


def test_parameter_file(tmp_path):
    path = tmp_path / 'params.yaml'
    path.write_text(PARAMS)
    expected = {'shared': 1, 'nested.value': (1.0, 2.0), 'own': 'x'}
    for _ in range(3):  # the cached file gives the same parameters every time
        assert _as_dict(
            module.to_parameters_list(LaunchContext(), 'my_node', '', [pathlib.Path(path)])
        ) == expected
    assert _as_dict(
        module.to_parameters_list(LaunchContext(), 'other_node', '', [pathlib.Path(path)])
    ) == {'shared': 1, 'nested.value': (1.0, 2.0), 'own': 'y'}


def test_parameter_file_parsed_once(tmp_path, monkeypatch):
    path = tmp_path / 'params.yaml'
    path.write_text(PARAMS)
    loads = []
    real_load = yaml.load

    def counting_load(stream, *args, **kwargs):
        # parameter values are parsed with yaml too: count the parameter file only
        if getattr(stream, 'name', None) == str(path):
            loads.append(1)
        return real_load(stream, *args, **kwargs)

    monkeypatch.setattr(yaml, 'load', counting_load)
    for node in ('my_node', 'other_node', 'my_node'):
        module.to_parameters_list(LaunchContext(), node, '', [pathlib.Path(path)])
    assert len(loads) == 1


def test_changed_parameter_file_is_parsed_again(tmp_path):
    path = tmp_path / 'params.yaml'
    path.write_text('/**:\n  ros__parameters:\n    a: 1\n')

    def value():
        (parameter,) = module.to_parameters_list(
            LaunchContext(), 'n', '', [pathlib.Path(path)])
        return parameter.value

    assert value() == 1
    path.write_text('/**:\n  ros__parameters:\n    a: 22\n')  # other size
    assert value() == 22
    stat = path.stat()
    path.write_text('/**:\n  ros__parameters:\n    a: 33\n')  # same size: the mtime moves on
    os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000))
    assert value() == 33
    os.unlink(path)
    with pytest.raises(OSError):
        value()
