# Copyright 2026 BYU FROST Lab
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

import os
from typing import Any

import yaml
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_comms_base_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")

    lead_agent_str = lead_agent.perform(context)
    agent_list_str = LaunchConfiguration("agent_list").perform(context)
    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    config_dir = os.environ["CONFIG_DIR"]

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_comms_params.yaml"]
    )
    scenario_param_file = scenario_param_path or fleet_param_file

    fleet_param_path = os.path.join(config_dir, "fleet", "coug_comms_params.yaml")

    poller_modem_frame = f"{lead_agent_str}/modem_link" if lead_agent_str else "base_station"

    dispatcher_modem_topics = {}
    poller_modem_topics = {}
    if lead_agent_str:
        dispatcher_modem_topics = {"modem_send_topic": f"/{lead_agent_str}/modem_send"}
        poller_modem_topics = {
            "modem_send_topic": f"/{lead_agent_str}/modem_send",
            "modem_rec_topic": f"/{lead_agent_str}/modem_rec",
            "modem_cmd_update_topic": f"/{lead_agent_str}/modem_cmd_update",
        }

    beacon_ids = {}
    for agent_ns in agent_list:
        agent_param_path = os.path.join(config_dir, f"{agent_ns}_params.yaml")
        launch_params = {
            **load_launch_params(fleet_param_path, "/**"),
            **load_launch_params(agent_param_path, f"/{agent_ns}"),
            **load_launch_params(scenario_param_path, "/**"),
            **load_launch_params(scenario_param_path, f"/{agent_ns}"),
        }
        beacon_id = launch_params.get("beacon_id")
        if beacon_id is not None:
            beacon_ids[agent_ns] = beacon_id

    return [
        Node(
            package="coug_comms",
            executable="base_dispatcher",
            name="base_dispatcher_node",
            parameters=[
                fleet_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "agent_list": agent_list,
                    "lead_agent": lead_agent,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
                    "beacon_ids": beacon_ids,
                    **dispatcher_modem_topics,
                },
            ],
        ),
        Node(
            package="coug_comms",
            executable="base_status_poller",
            name="base_status_poller_node",
            parameters=[
                fleet_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "agent_list": agent_list,
                    "lead_agent": lead_agent,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
                    "beacon_ids": beacon_ids,
                    "parameter_frame": poller_modem_frame,
                    **poller_modem_topics,
                },
            ],
        ),
        Node(
            package="coug_comms",
            executable="base_status_extractor",
            name="base_status_extractor_node",
            parameters=[
                fleet_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "agent_list": agent_list,
                    "map_frame": "map",
                    "multiagent_base_frame": "base_link",
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
            ),
            DeclareLaunchArgument(
                "enable_direct_comms",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_acoustic_comms",
                default_value="true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
