def usr(bot):
    """
    Minimal LED test for distributed mode signals.
    Bot 10 acts as 'watcher', all others listen for 'mode' messages.
    """

    # Track current LED color (percent values 0-100)
    current_led = [0, 0, 0]

    # Handler for mode messages broadcast via controller_send_mode.py
    def on_mode(_, msg):
        mode = msg.decode("utf-8").strip().lower()
        bot.logger.info(f"Received mode: {mode}")

        # Simple color mapping for each mode
        if mode == "float":
            current_led[:] = [0, 0, 100]          # blue
        elif mode == "glitch":
            current_led[:] = [100, 0, 100]        # magenta
        elif mode == "encircling":
            current_led[:] = [0, 100, 0]          # green
        elif mode == "stillness":
            current_led[:] = [100, 100, 100]      # white
        else:
            current_led[:] = [100, 0, 0]          # red = unknown mode

    # Subscribe to the 'mode' network slot
    bot.net.cctl.add_slot("mode", on_mode)

    # Main loop continually applies whatever LED color is current
    while True:
        bot.set_led(*current_led)
        bot.delay(200)  # refresh every 0.2 s so color stays solid
