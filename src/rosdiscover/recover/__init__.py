# -*- coding: utf-8 -*-
"""
This module is used to provide partial architecture recovery capabilities.
That is, it can be used to produce a (partial) model for a given node by
performing a simple static analysis of its associated source code. This code
operates under fairly heavy restrictions and is most successful when applied
to simple nodes with static models (rather than nodes that may publish and
subscribe to different topics at run-time depending on their configuration).
"""
