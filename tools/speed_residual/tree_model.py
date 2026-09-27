"""Pure-Python evaluator of a CatBoost oblivious-tree regressor exported as JSON.

No catboost at inference: a tree is a list of (feature index, border) and 2**depth leaf
values; split i sets bit i of the leaf index when x[feature] > border (CatBoost convention).
"""
from __future__ import annotations

import json


class ObliviousTrees:
    def __init__(self, path):
        m = json.loads(open(path, encoding='utf-8').read())
        self.trees = [([(s['float_feature_index'], s['border']) for s in t['splits']], t['leaf_values'])
                      for t in m['oblivious_trees']]
        scale, bias = m.get('scale_and_bias', [1.0, [0.0]])
        self.scale, self.bias = float(scale), float(bias[0])

    def __call__(self, x) -> float:
        total = 0.0
        for splits, leaves in self.trees:
            idx = 0
            for bit, (f, border) in enumerate(splits):
                if x[f] > border:
                    idx |= 1 << bit
            total += leaves[idx]
        return self.scale * total + self.bias
