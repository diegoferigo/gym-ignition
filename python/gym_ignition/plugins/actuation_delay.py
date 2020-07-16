# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import Tuple
from dataclasses import dataclass


@dataclass
class ActuationDelay:

    delay: int

    def to_xml(self) -> str:
        xml = f"""
        <sdf version='1.7'>
            <delay>{self.delay}</delay>
        </sdf>
        """

        return xml

    def args(self) -> Tuple[str, str, str]:
        return ("libActuationDelay.so",
                "scenario::plugins::gazebo::ActuationDelay",
                self.to_xml())
