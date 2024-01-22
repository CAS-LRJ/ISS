source ros1_ws/devel/setup.zsh
conda activate iss

export LEADERBOARD_ROOT=${ISS_ROOT}/ros1_ws/src/leaderboard
export SCENARIO_RUNNER_ROOT=${ISS_ROOT}/ros1_ws/src/scenario_runner

export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/:${SCENARIO_RUNNER_ROOT}:${LEADERBOARD_ROOT}

export ROUTES=${LEADERBOARD_ROOT}/data/routes_devtest.xml
export REPETITIONS=1
export DEBUG_CHALLENGE=1
export TEAM_AGENT=${ISS_ROOT}/ros1_ws/src/carla_bridge/scripts/carla_bridge/iss_leaderboard_agent.py
# export TEAM_AGENT=${LEADERBOARD_ROOT}/leaderboard/autoagents/ros1_agent.py
export CHECKPOINT_ENDPOINT=${LEADERBOARD_ROOT}/results.json
export CHALLENGE_TRACK_CODENAME=SENSORS

alias run_evaluation="zsh ${LEADERBOARD_ROOT}/scripts/run_evaluation.sh"
alias carla_launch="zsh $CARLA_ROOT/CarlaUE4.sh -quality-level=Low -windowed -resx=800 -resy=600"

