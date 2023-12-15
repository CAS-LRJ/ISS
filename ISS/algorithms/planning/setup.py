from setuptools import Extension

extensions = [Extension('dp_planner.dp_planner', ["dp_planner/dp_planner.pyx"]),
              Extension('dubins.dubins', ["dubins/dubins.pyx", "dubins/src/dubins.c"], include_dirs=[], language='c'),
              Extension('hybrid_a_star.hybrid_a_star', ["hybrid_a_star/hybrid_a_star.pyx"]),
              Extension('global_planner.lanelet2_planner', ["global_planner/lanelet2_planner.py"])]
