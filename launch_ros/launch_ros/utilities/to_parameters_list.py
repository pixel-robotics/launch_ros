# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module with utility to transform evaluated parameters into parameter lists."""

import copy
import os
import pathlib
import re
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple
import warnings

from launch.launch_context import LaunchContext
import rclpy.parameter

import yaml

from .evaluate_parameters import evaluate_parameter_dict
from .normalize_parameters import normalize_parameter_dict

from ..parameters_type import EvaluatedParameters


# libyaml-backed where available: ~6-9x faster than the pure-Python SafeLoader for the
# same result (rcl parses parameter files with libyaml as well)
_YAML_SAFE_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)

# path -> ((mtime_ns, size, inode), parsed file)
_parameter_file_cache: Dict[str, Tuple[Tuple[int, int, int], Any]] = {}


def _load_parameter_file(path: str) -> Any:
    """
    Return the parsed parameter file, parsed once while the file is unchanged.

    A launch with many composable nodes passes the same (often large) parameter files
    to each of them; they used to be parsed again for every node. The file is parsed
    again once its mtime, size or inode changed. Every call returns its own copy.
    """
    stat = os.stat(path)
    signature = (stat.st_mtime_ns, stat.st_size, stat.st_ino)
    cached = _parameter_file_cache.get(path)
    if cached is None or cached[0] != signature:
        with open(path, 'rb') as f:
            cached = (signature, yaml.load(f, Loader=_YAML_SAFE_LOADER))
        _parameter_file_cache[path] = cached
    return copy.deepcopy(cached[1])


def __normalize_parameters_dict(dictionary):
    """
    Combine namespaces and their node names into one key and remove the "ros__parameters" key.

    Return an empty dict if the "ros__parameters" key is not found.
    """
    # Recursive function
    def normalize_parameters_dict(dictionary, keys, result_dict):
        for key, value in dictionary.items():
            # Base case: found 'ros__parameters' key
            # Return
            if key == 'ros__parameters':
                node_name = '/'.join(keys)
                result_dict[node_name] = value
                return result_dict

            if isinstance(value, dict):
                keys.append(key.lstrip('/'))
                result_dict = normalize_parameters_dict(value, keys, result_dict)
                # Clean-up keys for the case of multiple nodes/ros__parameter entries
                keys.pop()

        return result_dict

    return normalize_parameters_dict(dictionary, [], {})


def is_node_name_matched(node_name: str, node_fqn: str) -> bool:
    # Will match ["/*" -> "(/\\w+)" and "/**" -> "(/\\w+)*"]
    regex = node_name.replace('/*', '(/\\w+)')
    return bool(re.fullmatch(regex, node_fqn))


def to_parameters_list(
    context: LaunchContext,
    node_name: str,
    namespace: str,
    evaluated_parameters: EvaluatedParameters
) -> List[rclpy.parameter.Parameter]:
    """
    Transform evaluated parameters into a list of rclpy.parameter.Parameter objects.

    :param context: to carry out substitutions during normalization of parameter files.
        See `normalize_parameters()` documentation for further reference.
    :param node_name: node name
    :param namespace: node namespace
    :param parameters: parameters as either paths to parameter files or name/value pairs.
        See `evaluate_parameters()` documentation for further reference.
    :returns: a list of parameters
    """
    parameters = []  # type: List[rclpy.parameter.Parameter]
    node_name = node_name.lstrip('/')
    if namespace and namespace != '/':
        namespace = namespace.strip('/')
        node_name = f'{namespace}/{node_name}'

    params_set = {}
    warned_once = False
    for params_set_or_path in evaluated_parameters:
        if isinstance(params_set_or_path, pathlib.Path):
            param_dict = _load_parameter_file(str(params_set_or_path))
            normalized_param_dict = __normalize_parameters_dict(param_dict)

            if normalized_param_dict:
                param_dict.clear()
                for key in normalized_param_dict:
                    if (is_node_name_matched(f'/{key}', f'/{node_name}')):
                        param_dict.update(normalized_param_dict[key])
                if node_name in normalized_param_dict:
                    param_dict.update(normalized_param_dict[node_name])
                if not warned_once and not node_name:
                    warnings.warn(
                        'node name not provided to launch; parameter files will not apply',
                        UserWarning
                    )
                    warned_once = True

            if param_dict:
                params_set = evaluate_parameter_dict(
                    context, normalize_parameter_dict(param_dict)
                )
        else:
            params_set = params_set_or_path
        if not isinstance(params_set, dict):
            raise RuntimeError('invalid evaluated parameters {}'.format(repr(params_set)))
        for name, value in params_set.items():
            type_ = next((
                type_ for type_ in rclpy.parameter.Parameter.Type if type_.check(value)
            ), None)  # type: rclpy.parameter.Parameter.Type
            if type_ is None:
                raise RuntimeError('invalid parameter value {}'.format(repr(value)))
            parameters.append(rclpy.parameter.Parameter(name, type_, value))
    return parameters
