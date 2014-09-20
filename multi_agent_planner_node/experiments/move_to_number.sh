#!/bin/bash
move_env="env_$1.yaml"
echo "moving env to $move_env"
mv "env.yaml" "$move_env"

echo "moving experiments to swarm_tests_$1.yaml"
mv "swarm_tests.yaml" "swarm_tests_$1.yaml"
