# usr_code_mode_echo.py
def usr(bot):
    current = b""
    def on_mode(_, payload: bytes):
        nonlocal current
        current = payload.strip() or b""
        bot.logger.info(f"MODE={current!r}")

    # listen for PC signals named "mode"
    bot.net.cctl.add_slot("mode", on_mode)

    # simple visual feedback loop
    while True:
        # set LED based on mode name
        if current == b"glitch":
            bot.set_led(30, 0, 30)
        elif current == b"directional_left":
            bot.set_led(0, 30, 0)
        elif current == b"directional_right":
            bot.set_led(0, 0, 30)
        elif current == b"encircling":
            bot.set_led(30, 30, 0)
        elif current == b"glide":
            bot.set_led(20, 20, 20)
        else:
            bot.set_led(5, 5, 5)  # idle/unknown
        bot.delay(0.1)
