# MESSAGE FROM RISHI PLEASE READ THIS FUTURE TEAM MEMBER
# ~=[,,_,,]:3
# Alright, so basically, the robot is programmed to sweep the L/Z in an up-right-down-left-up lawnmower type path.
# It will travel forward (up or down) by whatever amount you choose, I just chose 10 forward movements till it turns
# in the move_and_scan function, it will move forward, scan ultrasonic and lidar in the front, slightly left and slightly right, it will then do some basic logic to see if the block is there
# I call on the scan_rplidar and send_command functions Baqir wrote in his scan_processing.py file
# Lastly, this is for Baqir, I need you to write three new arduino commands called obs_forwardskip that basically moves the robot forward twice, obs_rotateright90 that turns the robot right by 90 degrees and obs_rotateleft90 that turns robot left by 90 degrees

LAWN_MOWER_LENGTH = 10  # Number of forward movements in for vertical sweep
LIDAR_ULTRASONIC_DIFF_THRESHOLD = 2  # Maximum tolerance between LiDAR and ultrasonic readings

def move_and_scan(server_ip, port):
    """
    Moves the robot forward and performs a sweep scan for block detection.
    
    Args:
        server_ip (str): IP address of the robot server.
        port (int): Port for communication.
    
    Returns:
        bool: True if a block is detected, False otherwise.
    """
    # Initial LIDAR scan and ultrasonic front scan
    front_ultrasonic = float(send_command(server_ip, port, command="scan"))
    lidar_front_initial = scan_rplidar()[0]

    # Perform small sweeping movements (left and right) to get LIDAR scans
    send_command(server_ip, port, command="obs_smallrotateLeft")
    left_scan = float(send_command(server_ip, port, command="scan"))
    lidar_left = scan_rplidar()[0]

    send_command(server_ip, port, command="obs_smallrotateRight")
    right_scan = float(send_command(server_ip, port, command="scan"))
    lidar_right = scan_rplidar()[0]

    # Block detection: Compare front, left, and right LIDAR scans with ultrasonic
    block_detected_count = 0  # Counter to track detected blocks

    for position, scan, lidar in [("front", front_ultrasonic, lidar_front_initial),
                                  ("left", left_scan, lidar_left),
                                  ("right", right_scan, lidar_right)]:
        if scan < 10.0 and lidar - scan > LIDAR_ULTRASONIC_DIFF_THRESHOLD:
            print(f"Potential block detected at {position} - Distances - {position.capitalize()}: {scan}, LiDAR: {lidar}")
            rescan = float(send_command(server_ip, port, command="scan"))
            lidar_rescan = scan_rplidar()[0]

            if rescan < 5.0 and lidar_rescan - rescan > LIDAR_ULTRASONIC_DIFF_THRESHOLD:
                print(f"Block confirmed at {position} - Distances - {position.capitalize()}: {rescan}, LiDAR: {lidar_rescan}")
                block_detected_count += 1

    # Return True if at least two positions detect a block
    return block_detected_count >= 2

def block_detection_lawnmower(server_ip, port):
    """
    Simple lawnmower pattern for obstacle avoidance and block detection.
    
    Args:
        server_ip (str): IP address of the robot server.
        port (int): Port for communication.
    
    Returns:
        bool: True if a block is detected, False otherwise.
    """
    Block_detected = False

    while not Block_detected:
        # Step 1: Move forward for the lawnmower length
        for _ in range(LAWN_MOWER_LENGTH):
            send_command(server_ip, port, command="obs_moveForward")
            
            # Perform a sweep scan for block detection
            if move_and_scan(server_ip, port):
                Block_detected = True
                return True
        
        # Step 2: Turn right 90 degrees
        send_command(server_ip, port, command="obs_rotateright90")
        
        # Step 3: Move forward twice
        send_command(server_ip, port, command="obs_forwardskip")
        
        # Step 4: Turn right 90 degrees again to face south
        send_command(server_ip, port, command="obs_rotateright90")
        
        # Step 5: Move forward for the lawnmower length (southward strip)
        for _ in range(LAWN_MOWER_LENGTH):
            send_command(server_ip, port, command="obs_moveForward")
            
            # Perform a sweep scan for block detection
            if move_and_scan(server_ip, port):
                Block_detected = True
                return True
        
        # Step 6: Turn left 90 degrees
        send_command(server_ip, port, command="obs_rotateleft90")
        
        # Step 7: Move forward three times to adjust for next strip
        send_command(server_ip, port, command="obs_forwardskip")
        
        # Step 8: Turn left 90 degrees again to face north
        send_command(server_ip, port, command="obs_rotateleft90")
    
    return False