#!/bin/bash
# Smart Campus Delivery - Navigation Test
# Simulates autonomous patrol between Building A → B → C
# Tests waypoint navigation for package delivery routes

echo "=========================================="
echo "Campus Delivery - Navigation Test"
echo "=========================================="
echo "Testing: Autonomous building-to-building patrol"
echo "Route: Building A → Building B → Building C"
echo ""
echo "Building Locations:"
echo "  🏛️  Building A (Engineering): x=0.0, y=0.0"
echo "  📚 Building B (Library): x=3.0, y=2.0"
echo "  🍽️  Building C (Cafeteria): x=6.0, y=0.5"
echo ""
echo "This simulates the delivery robot navigating between buildings"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Array of buildings
buildings=("Building A (Engineering)" "Building B (Library)" "Building C (Cafeteria)")
coords=("x=0.0, y=0.0" "x=3.0, y=2.0" "x=6.0, y=0.5")
building_index=0
delivery_count=0

# Function to publish navigation status
publish_status() {
    local message=$1
    ros2 topic pub --once campus_delivery/navigation/status std_msgs/String "{data: '$message'}" 2>/dev/null
}

# Function to publish current building
publish_building() {
    local building=$1
    ros2 topic pub --once campus_delivery/current_building std_msgs/String "{data: '$building'}" 2>/dev/null
}

echo "🤖 Starting campus delivery patrol..."
echo ""

while true; do
    # Current building
    current_building="${buildings[$building_index]}"
    current_coords="${coords[$building_index]}"
    
    # Next building (wrap around)
    next_index=$(( (building_index + 1) % 3 ))
    next_building="${buildings[$next_index]}"
    next_coords="${coords[$next_index]}"
    
    # At current building
    echo "📍 Current Location: $current_building ($current_coords)"
    publish_building "$current_building"
    publish_status "Arrived at $current_building"
    echo "   📦 Delivering packages..."
    sleep 2
    
    delivery_count=$((delivery_count + 1))
    echo "   ✓ Delivery #$delivery_count complete!"
    echo ""
    
    # Navigate to next building
    echo "🚚 En route to: $next_building ($next_coords)"
    publish_status "En route to $next_building"
    
    # Simulate navigation time (distance-dependent)
    nav_time=5
    for ((i=1; i<=nav_time; i++)); do
        remaining=$((nav_time - i + 1))
        distance=$(echo "scale=1; $remaining * 1.2" | bc)
        echo "   Navigating... (${distance}m remaining)"
        publish_status "Distance remaining: ${distance}m"
        sleep 1
    done
    
    echo "   ✓ Reached $next_building"
    echo ""
    echo "=========================================="
    echo ""
    
    # Move to next building
    building_index=$next_index
    
    sleep 2
done
