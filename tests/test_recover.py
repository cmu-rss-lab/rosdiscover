# -*- coding: utf-8 -*-
import pytest


@pytest.mark.parametrize('config', ['fetch'], indirect=True)
def test_follow_waypoints(config) -> None:
    recover = NodeRecoveryTool.for_config(config)
