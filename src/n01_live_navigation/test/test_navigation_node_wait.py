"""Graph verification rejects duplicates, normalizes namespaces and accepts order."""
import importlib.util
from pathlib import Path
p=Path(__file__).resolve().parents[3]/'scripts/n01_wait_navigation_nodes.py'
s=importlib.util.spec_from_file_location('node_wait_probe',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

def test_graph_names_are_normalized_and_unrelated_nodes_do_not_block():
    assert m.missing_nodes([('planner','/'),('bridge','/n01'),('extra','/')],['/planner','/n01/bridge'])=={}

def test_graph_missing_and_duplicate_counts_are_reported():
    assert m.missing_nodes([('planner','/'),('planner','/')],['/planner','/bridge'])=={'/planner':2,'/bridge':0}
