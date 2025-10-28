import time
import struct

LEADER_ID = 0  # Bot 0 is the leader/listener

# Message format for bot-to-bot communication (like MSI example)
MSG_FMT = 'iffffiii'  # ID, x, y, dx, dy, led_r, led_g, led_b

def decode_message(message):
    """EXACT MSI example decode function"""
    decoded = message.decode('ascii')
    speed_msg, led_msg = tuple(decoded.split(';'))
    speed = [float(v) for v in speed_msg.split(',')]
    led = [int(float(v) / 255 * 100) for v in led_msg.split(',')]  # Convert 255→100
    return speed[0], speed[1], led

def copy_list(in_l, out_l):
    """EXACT MSI example copy function"""
    assert len(in_l) == len(out_l)
    for i, val in enumerate(in_l):
        out_l[i] = val

def usr(bot):
    """Main function - MSI structure with leader/follower roles"""
    bot.logger.info("Bot %d starting MSI-style controller" % bot.id)
    
    # Boot sequence
    for _ in range(2):
        bot.set_led(100, 100, 100)
        time.sleep(0.2)
        bot.set_led(0, 0, 0)
        time.sleep(0.2)
    
    # State variables (like MSI example)
    current_dir = [0.0, 0.0]
    current_led = [0, 0, 0]
    
    def message_handler(_, message):
        """Handler for controller messages (leader only)"""
        bot.logger.info("Leader received controller message")
        dir_x, dir_y, led = decode_message(message)
        copy_list([dir_x, dir_y], current_dir)
        copy_list(led, current_led)
        
        # Leader sets its own LED immediately
        bot.set_led(current_led[0], current_led[1], current_led[2])
        bot.logger.info("Leader LED set to %s" % current_led)
    
    # Only leader registers for controller messages
    if bot.id == LEADER_ID:
        bot.net.add_slot('speed_led', message_handler)
        bot.logger.info("Leader registered for controller messages")
    
    # Main loop - different behavior for leader vs followers
    while True:
        if bot.id == LEADER_ID:
            # LEADER: Broadcast current state to followers in radius
            current_pos, _ = bot.get_pose_blocking(1.0)
            try:
                # Broadcast message to all bots in communication radius
                bot.send_msg(struct.pack(MSG_FMT,
                    bot.id, current_pos.x, current_pos.y,
                    0, 0,  # No movement
                    current_led[0], current_led[1], current_led[2]))
                bot.logger.info("Leader broadcasting LED: %s" % current_led)
            except Exception as e:
                bot.logger.info("Leader broadcast error: %s" % str(e))
                
        else:
            # FOLLOWER: Listen for leader broadcasts
            try:
                msgs = bot.recv_msg()
                if msgs:
                    for msg_raw in msgs:
                        if len(msg_raw) >= 32:  # Minimum message size
                            try:
                                message = struct.unpack(MSG_FMT, msg_raw[:32])
                                # Only listen to leader
                                if message[0] == LEADER_ID:
                                    new_led = [message[5], message[6], message[7]]
                                    copy_list(new_led, current_led)
                                    bot.set_led(current_led[0], current_led[1], current_led[2])
                                    bot.logger.info("Follower %d updated LED: %s" % (bot.id, current_led))
                            except Exception as e:
                                bot.logger.info("Follower message error: %s" % str(e))
            except Exception as e:
                bot.logger.info("Follower receive error: %s" % str(e))
        
        time.sleep(0.1)