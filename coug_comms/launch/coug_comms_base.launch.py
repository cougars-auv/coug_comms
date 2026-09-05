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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    lead_agent = LaunchConfiguration("lead_agent")
    lead_agent_ns = lead_agent.perform(context)
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_comms_params.yaml",
        ]
    )

    poller_modem_frame = PythonExpression(
        [
            "'",
            lead_agent,
            "/modem_link' if '",
            lead_agent,
            "' != '' else 'base_station'",
        ]
    )

    dispatcher_modem_topics = {}
    poller_modem_topics = {}
    if lead_agent_ns:
        dispatcher_modem_topics = {"modem_send_topic": f"/{lead_agent_ns}/modem_send"}
        poller_modem_topics = {
            "modem_send_topic": f"/{lead_agent_ns}/modem_send",
            "modem_rec_topic": f"/{lead_agent_ns}/modem_rec",
            "modem_cmd_update_topic": f"/{lead_agent_ns}/modem_cmd_update",
        }

    config_dir = os.environ["CONFIG_DIR"]

    def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
        try:
            with open(path) as config_file:
                config = yaml.safe_load(config_file)
            return config[top_key]["coug_comms_base_launch"]["ros__parameters"]
        except (KeyError, TypeError, OSError):
            return {}

    fleet_defaults = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_comms_params.yaml"), "/**"
    )
    beacon_ids = {}
    for agent_ns in agent_list:
        agent_launch_params = load_launch_params(
            os.path.join(config_dir, f"{agent_ns}_params.yaml"), f"/{agent_ns}"
        )
        beacon_id = agent_launch_params.get(
            "beacon_id", fleet_defaults.get("beacon_id")
        )
        if beacon_id is not None:
            beacon_ids[agent_ns] = beacon_id

    return [
        Node(
            package="coug_comms",
            executable="base_dispatcher",
            name="base_dispatcher_node",
            parameters=[
                fleet_param_file,
                {
                    "agent_list": agent_list,
                    "beacon_ids": beacon_ids,
                    "use_sim_time": use_sim_time,
                    "lead_agent": lead_agent,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
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
                {
                    "agent_list": agent_list,
                    "beacon_ids": beacon_ids,
                    "use_sim_time": use_sim_time,
                    "lead_agent": lead_agent,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
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
                {
                    "agent_list": agent_list,
                    "use_sim_time": use_sim_time,
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
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
                description="Namespace of the lead agent (optional)",
            ),
            DeclareLaunchArgument(
                "enable_direct_comms",
                default_value="true",
                description="Enable direct ROS service communications",
            ),
            DeclareLaunchArgument(
                "enable_acoustic_comms",
                default_value="true",
                description="Enable acoustic communications",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
