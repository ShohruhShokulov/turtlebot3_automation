#!/bin/bash
# Smart Campus Delivery - Maintenance Health Monitoring Test
# Simulates battery drain and health monitoring for delivery robot
# This prevents mid-route failures during campus package delivery

echo "=========================================="
echo "Campus Delivery - Maintenance Test"
echo "=========================================="
echo "Testing: Battery & health monitoring"
echo "Purpose: Prevent delivery robot failures mid-route"
echo ""
echo "This simulates battery data for the delivery robot"
echo "Battery will drain from 12.4V → 10.5V (critical)"
echo "Alerts will trigger when battery gets too low"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Start with healthy battery for campus delivery
voltage=12.4
percentage=1.0
delivery_count=0

while true; do
    # Publish battery state
    ros2 topic pub --once /battery_state sensor_msgs/BatteryState \
        "{voltage: $voltage, percentage: $percentage, power_supply_status: 1, present: true}"
    
    # Calculate battery percentage as integer
    pct=$(echo "scale=0; $percentage * 100" | bc)
    
    echo "📦 Delivery Robot Status: ${voltage}V (${pct}%) | Deliveries: $delivery_count"
    
    # Simulate delivery completion
    delivery_count=$((delivery_count + 1))
    
    # Drain battery slightly with each delivery
    voltage=$(echo "$voltage - 0.05" | bc)
    percentage=$(echo "$percentage - 0.02" | bc)
    
    # Check battery status
    if (( $(echo "$voltage <= 10.8" | bc -l) )); then
        echo "🚨 CRITICAL: Battery too low for delivery! Return to charging station!"
    elif (( $(echo "$voltage <= 11.4" | bc -l) )); then
        echo "⚠️  WARNING: Battery low - Complete current delivery and recharge"
    fi
    
    # Reset if too low (simulate recharge)
    if (( $(echo "$voltage < 10.5" | bc -l) )); then
        echo ""
        echo "🔌 Robot returned to charging station and fully recharged"
        echo "=========================================="
        voltage=12.4
        percentage=1.0
        delivery_count=0
    fi
    
    sleep 5
done
