#!/bin/bash

# Test script for process separation
# Runs simulator in background, then runs agent client, then cleans up

echo "========================================="
echo "Testing FlатSim Process Separation"
echo "========================================="

# Start simulator server in background
echo "Starting simulator server..."
./build/simulator_server &
SIM_PID=$!
echo "Simulator PID: $SIM_PID"

# Wait for simulator to initialize
sleep 2

# Run agent client
echo ""
echo "Starting agent client..."
./build/agent_client examples/machines/tractor.json

# Clean up
echo ""
echo "Cleaning up simulator..."
kill $SIM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null

echo ""
echo "Test complete!"
