#!/bin/bash
# Test the pattern system integration safely

echo "╔══════════════════════════════════════════════════════╗"
echo "║    PATTERN INTEGRATION TEST (SAFE)                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This demonstrates how patterns can work with LLM"
echo "using a separate bridge node (no modifications to working code)"
echo ""
echo "Prerequisites:"
echo "  - Full system running (./launch_hybrid_system.sh)"
echo "  - Pattern nodes running (./test_patterns_standalone.sh)"
echo ""

# Test expanding square via bridge
echo "1. Testing expanding square pattern via LLM bridge:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:expanding_square'\" --once"
echo ""

# Test spiral via bridge  
echo "2. Testing spiral pattern via LLM bridge:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:spiral'\" --once"
echo ""

# Stop pattern
echo "3. Stop pattern:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'stop_pattern'\" --once"
echo ""

echo "NOTE: This uses a separate topic (/llm/pattern_request)"
echo "      so it doesn't interfere with existing /llm/command_input"
echo ""
echo "When ready to integrate, we can add pattern recognition"
echo "to the practical_llm_controller safely."
echo ""