"""The defense algorithms must never read simulation ground truth."""

import os
import re

PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'perimeter_defense')
FORBIDDEN = [r'/sim/', r'ground_truth', r'/world/[^\s\'"]*/pose', r'dynamic_pose', r'/actor/']


def test_no_ground_truth_topics():
    for root, _, files in os.walk(PKG):
        for f in files:
            if f.endswith('.py'):
                text = open(os.path.join(root, f), encoding='utf-8').read()
                for pattern in FORBIDDEN:
                    assert not re.search(pattern, text), f'{f} references {pattern}'
