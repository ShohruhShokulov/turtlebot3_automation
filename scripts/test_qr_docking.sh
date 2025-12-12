#!/bin/bash
# Smart Campus Delivery - QR-Docking System Test
# Simulates QR code detection on room doors for automatic docking
# Tests room identification and docking for package delivery

echo "=========================================="
echo "Campus Delivery - QR-Docking System Test"
echo "=========================================="
echo "Testing: QR code-based room docking"
echo "Purpose: Auto-dock at correct room for delivery"
echo ""
echo "Room QR Codes:"
echo "  🚪 ROOM_A → Engineering Lab 101"
echo "  🚪 ROOM_B → Library Office 205"
echo "  🚪 ROOM_C → Faculty Lounge 310"
echo ""
echo "This simulates QR code detection and auto-docking sequence"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Array of rooms
rooms=("ROOM_A" "ROOM_B" "ROOM_C")
room_names=("Engineering Lab 101" "Library Office 205" "Faculty Lounge 310")
room_index=0
delivery_count=0

# Function to publish QR detected
publish_qr() {
    local qr_code=$1
    ros2 topic pub --once --wait-matching-subscriptions 0 campus_delivery/qr_detected std_msgs/String "{data: '$qr_code'}" >/dev/null 2>&1
}

# Function to publish docking status
publish_docking() {
    local status=$1
    ros2 topic pub --once --wait-matching-subscriptions 0 campus_delivery/docking_status std_msgs/String "{data: '$status'}" >/dev/null 2>&1
}

echo "🤖 Starting QR-docking demonstration..."
echo ""

while true; do
    # Current room
    room_qr="${rooms[$room_index]}"
    room_name="${room_names[$room_index]}"
    
    # Scanning for QR code
    echo "📸 Camera scanning for room QR codes..."
    sleep 2
    
    # QR code detected!
    echo "✓ QR CODE DETECTED: $room_qr"
    echo "  Room: $room_name"
    publish_qr "$room_qr"
    echo ""
    sleep 1
    
    # Approaching door
    echo "🚚 Initiating docking sequence for $room_qr..."
    echo "  Approaching door..."
    
    # Simulate approach (5 steps)
    for ((i=5; i>=1; i--)); do
        distance=$(echo "scale=1; $i * 0.3" | bc)
        echo "  └─ Distance to door: ${distance}m"
        sleep 1
    done
    
    echo "  └─ Aligning with door..."
    sleep 1
    
    # Docked!
    echo ""
    echo "✓ DOCKED at $room_qr ($room_name)!"
    publish_docking "DOCKED:$room_qr"
    echo ""
    
    delivery_count=$((delivery_count + 1))
    echo "📦 Delivery #$delivery_count"
    echo "  ├─ Package delivered to: $room_name"
    echo "  ├─ Recipient notified"
    echo "  └─ Delivery confirmed"
    echo ""
    sleep 2
    
    # Undock and prepare for next delivery
    echo "🔄 Undocking from $room_qr..."
    publish_docking "UNDOCKED:$room_qr"
    sleep 1
    echo "  Ready for next delivery"
    echo ""
    echo "=========================================="
    echo ""
    
    # Move to next room
    room_index=$(( (room_index + 1) % 3 ))
    
    sleep 3
done
